#include "gcode_planner.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

double sg_segment_seconds(const sg_machine *machine, const double *first,
                          const double *second, double nominal) {
    double seconds = nominal;
    /* Rest-to-rest quintic: s=10u^3-15u^4+6u^5. Exact maxima of |s'|,
       |s''|, |s'''| are 1.875, 10*sqrt(3)/3, and 60. */
    for (int axis = 0; axis < machine->count; ++axis) {
        double displacement = fabs(second[axis] - first[axis]);
        seconds = fmax(seconds, 1.875 * displacement / machine->axes[axis].velocity);
        seconds = fmax(seconds, sqrt((10 * sqrt(3) / 3) * displacement / machine->axes[axis].acceleration));
        seconds = fmax(seconds, cbrt(60 * displacement / machine->axes[axis].jerk));
    }
    return seconds * (1 + 1e-9);
}

typedef struct { int count; sg_configuration configurations[SG_CANDIDATES]; } candidate_layer;
typedef struct {
    int candidate, parent;
    double cost, seconds;
    double average_velocity[SG_AXES], acceleration_score[SG_AXES];
} search_node;

static int compare_nodes(const void *first, const void *second) {
    const search_node *left = first, *right = second;
    if (left->cost < right->cost) return -1;
    if (left->cost > right->cost) return 1;
    if (left->candidate != right->candidate) return left->candidate < right->candidate ? -1 : 1;
    return left->parent < right->parent ? -1 : left->parent > right->parent;
}

int sg_plan_path(const sg_machine *machine, const sg_toolpath *path,
                 sg_plan *plan, sg_diagnostic *diagnostic) {
    if (path->count < 2 || path->count > SG_POINTS || (machine->count != 5 && machine->count != 12))
        return sg_fail(diagnostic, SG_INPUT, 0, "invalid path or unsupported machine model");
    candidate_layer *layers = calloc((size_t)path->count, sizeof *layers);
    search_node *history = calloc((size_t)path->count * SG_BEAM, sizeof *history);
    search_node *options = calloc(SG_BEAM * SG_CANDIDATES, sizeof *options);
    signed char *edge_cache = malloc(SG_CANDIDATES * SG_CANDIDATES);
    int *beam_counts = calloc((size_t)path->count, sizeof *beam_counts);
    if (!layers || !history || !options || !edge_cache || !beam_counts) {
        free(layers); free(history); free(options); free(edge_cache); free(beam_counts);
        return sg_fail(diagnostic, SG_CAPACITY, 0, "planner allocation failed");
    }
    int result = SG_OK;
    memset(plan, 0, sizeof *plan);
    for (int point = 0; point < path->count; ++point) {
        layers[point].count = sg_inverse_candidates(machine,path,point,layers[point].configurations,diagnostic);
        if (layers[point].count < 0) { result = diagnostic->code; goto finish; }
    }
    history[0].candidate = 0; history[0].parent = -1; beam_counts[0] = 1;
    for (int point = 1; point < path->count; ++point) {
        int option_count = 0;
        memset(edge_cache, -1, SG_CANDIDATES * SG_CANDIDATES);
        double singular_values[SG_CANDIDATES];
        for (int candidate = 0; candidate < layers[point].count; ++candidate)
            singular_values[candidate] = sg_analyze(machine,layers[point].configurations[candidate].joint,0,0).minimum_singular_value;
        for (int parent = 0; parent < beam_counts[point-1]; ++parent) {
            search_node *previous = &history[(point-1)*SG_BEAM+parent];
            const double *start = layers[point-1].configurations[previous->candidate].joint;
            for (int candidate = 0; candidate < layers[point].count; ++candidate) {
                const double *end = layers[point].configurations[candidate].joint;
                int cache_index = previous->candidate * SG_CANDIDATES + candidate;
                if (edge_cache[cache_index] < 0)
                    edge_cache[cache_index] = (signed char)sg_edge_valid(machine,&path->points[point-1],&path->points[point],start,end,NULL);
                if (!edge_cache[cache_index]) continue;
                search_node next = {0};
                next.candidate = candidate; next.parent = parent;
                next.seconds = sg_segment_seconds(machine,start,end,path->points[point].seconds);
                if (!isfinite(next.seconds) || next.seconds > 1e4) continue;
                next.cost = previous->cost + 0.0001*next.seconds + 0.0001/(0.02+singular_values[candidate]);
                for (int axis = 0; axis < machine->count; ++axis) {
                    double displacement = end[axis]-start[axis];
                    double scale = machine->axes[axis].rotary ? 1 : machine->tool_length;
                    next.average_velocity[axis] = displacement/next.seconds;
                    double separation = point > 1 ? (next.seconds+previous->seconds)/2 : next.seconds;
                    next.acceleration_score[axis] = (next.average_velocity[axis]-previous->average_velocity[axis])/separation;
                    double jerk_score = (next.acceleration_score[axis]-previous->acceleration_score[axis])/separation;
                    next.cost += fabs(displacement)/scale;
                    next.cost += 0.01*pow(next.average_velocity[axis]/machine->axes[axis].velocity,2);
                    next.cost += 0.01*pow(next.acceleration_score[axis]/machine->axes[axis].acceleration,2);
                    next.cost += 0.01*pow(jerk_score/machine->axes[axis].jerk,2);
                    double width = machine->axes[axis].upper-machine->axes[axis].lower;
                    double margin = fmin(end[axis]-machine->axes[axis].lower,machine->axes[axis].upper-end[axis])/width;
                    next.cost += 1e-5/(0.01+fmax(0,margin));
                }
                options[option_count++] = next;
            }
        }
        if (!option_count) {
            result = sg_fail(diagnostic, SG_NO_CONTINUATION, point,
                "no certified connection from the retained search states; add explicit toolpath detail or broaden redundant sampling");
            goto finish;
        }
        qsort(options,(size_t)option_count,sizeof *options,compare_nodes);
        /* Retain at most one history per (last candidate, previous candidate).
           Full-path beam search is deliberately bounded; no optimality or
           continuous-fiber completeness theorem is claimed. */
        for (int option = 0; option < option_count && beam_counts[point] < SG_BEAM; ++option) {
            int duplicate = 0;
            for (int kept = 0; kept < beam_counts[point]; ++kept) {
                search_node *other = &history[point*SG_BEAM+kept];
                if (other->candidate == options[option].candidate &&
                    history[(point-1)*SG_BEAM+other->parent].candidate == history[(point-1)*SG_BEAM+options[option].parent].candidate) duplicate = 1;
            }
            if (!duplicate) history[point*SG_BEAM+beam_counts[point]++] = options[option];
        }
    }
    plan->count = path->count;
    plan->cost = history[(path->count-1)*SG_BEAM].cost;
    int selected = 0;
    for (int point = path->count-1; point >= 0; --point) {
        search_node *node = &history[point*SG_BEAM+selected];
        plan->configuration[point] = layers[point].configurations[node->candidate];
        plan->seconds[point] = node->seconds;
        selected = node->parent;
    }
    result = sg_validate_plan(machine,path,plan,diagnostic);
finish:
    free(layers); free(history); free(options); free(edge_cache); free(beam_counts);
    return result;
}

int sg_validate_plan(const sg_machine *machine, const sg_toolpath *path,
                     const sg_plan *plan, sg_diagnostic *diagnostic) {
    if (plan->count != path->count || plan->count < 2)
        return sg_fail(diagnostic, SG_INPUT, 0, "plan and toolpath sizes disagree");
    for (int axis = 0; axis < machine->count; ++axis)
        if (fabs(plan->configuration[0].joint[axis]-machine->axes[axis].initial) > 1e-8)
            return sg_fail(diagnostic, SG_INPUT, 0, "plan initial joints disagree with machine precondition");
    for (int point = 1; point < path->count; ++point) {
        if (!isfinite(plan->seconds[point]) || plan->seconds[point] <= 0 || plan->seconds[point] > 1e4)
            return sg_fail(diagnostic, SG_INPUT, point, "invalid planned segment time");
        double required = sg_segment_seconds(machine,plan->configuration[point-1].joint,
            plan->configuration[point].joint,path->points[point].seconds)/(1+1e-9);
        if (plan->seconds[point]+1e-8 < required)
            return sg_fail(diagnostic, SG_INPUT, point, "plan timing violates the reference velocity, acceleration or jerk bound");
        if (!sg_edge_valid(machine,&path->points[point-1],&path->points[point],
            plan->configuration[point-1].joint,plan->configuration[point].joint,diagnostic)) {
            diagnostic->point = point;
            return diagnostic->code;
        }
    }
    return SG_OK;
}

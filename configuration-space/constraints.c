#include "gcode_planner.h"
#include <math.h>

static double box_distance(sg_vector point, sg_vector half_size) {
    sg_vector outside = {fabs(point.x)-half_size.x, fabs(point.y)-half_size.y, fabs(point.z)-half_size.z};
    sg_vector positive = {fmax(0,outside.x),fmax(0,outside.y),fmax(0,outside.z)};
    return sg_length(positive) + fmin(0, fmax(outside.x, fmax(outside.y,outside.z)));
}

/* A conservative 1-Lipschitz clearance lower bound for the finished fitting:
   rectangular body minus an X through bore and a tilted, upward branch ray.
   This is a clearance model, not a material-removal simulation. */
static double fitting_distance(const sg_machine *machine, sg_vector point) {
    double outside_stock = box_distance(point, machine->stock_half_size);
    double inside_main_bore = machine->main_bore_radius - hypot(point.y, point.z);
    sg_vector branch = {sin(machine->branch_angle), 0, cos(machine->branch_angle)};
    double along = fmax(0, sg_dot(point, branch));
    double inside_branch = machine->branch_bore_radius - sg_length(sg_subtract(point, sg_scale(branch, along)));
    return fmax(outside_stock, fmax(inside_main_bore, inside_branch));
}

static double segment_distance(sg_vector point, sg_vector first, sg_vector second) {
    sg_vector difference = sg_subtract(second, first);
    double denominator = sg_dot(difference, difference);
    double parameter = denominator > 0 ? sg_dot(sg_subtract(point, first), difference) / denominator : 0;
    parameter = fmax(0, fmin(1, parameter));
    return sg_length(sg_subtract(point, sg_add(first, sg_scale(difference, parameter))));
}

double sg_clearance(const sg_machine *machine, const double *joint) {
    sg_pose pose = sg_forward(machine, joint);
    sg_rotation head = sg_rotation_product(sg_rotate_z(joint[4]), sg_rotate_y(joint[3]));
    sg_rotation table = machine->count == 12 ? sg_rotation_product(sg_rotate_z(joint[11]), sg_rotate_x(joint[10])) : sg_rotate_x(0);
    sg_vector origin = machine->count == 12 ? (sg_vector){joint[8],joint[9],0} : (sg_vector){0};
    sg_vector tip_world = sg_add(origin, sg_rotate(table, pose.position));
    sg_vector nose_world = sg_add(tip_world, sg_scale(sg_rotate(head,(sg_vector){0,0,1}), machine->tool_length));
    sg_vector housing_world = sg_add(nose_world, sg_rotate(head,(sg_vector){machine->housing_offset,0,0}));
    sg_vector nose_work = sg_add(pose.position, sg_scale(pose.direction, machine->tool_length));
    sg_vector housing_work = sg_add(nose_work, sg_rotate(pose.rotation,(sg_vector){machine->housing_offset,0,0}));
    double clearance = 1e6;
    for (int obstacle = 0; obstacle < machine->obstacle_count; ++obstacle) {
        sg_sphere sphere = machine->obstacles[obstacle];
        clearance = fmin(clearance, segment_distance(sphere.center, tip_world, nose_world) - sphere.radius - machine->tool_radius);
        clearance = fmin(clearance, sg_length(sg_subtract(sphere.center, nose_world)) - sphere.radius - machine->head_radius);
        clearance = fmin(clearance, sg_length(sg_subtract(sphere.center, housing_world)) - sphere.radius - machine->head_radius);
    }
    if (machine->fitting) {
        /* Sampling plus a Lipschitz spacing bound certifies the whole capsule,
           rather than claiming that sampled tool points alone prove clearance. */
        const int subdivisions = 128;
        double spacing_margin = machine->tool_length / (2 * subdivisions);
        for (int sample = 0; sample <= subdivisions; ++sample) {
            sg_vector point = sg_add(pose.position, sg_scale(pose.direction, machine->tool_length * sample / subdivisions));
            clearance = fmin(clearance, fitting_distance(machine, point) - machine->tool_radius - spacing_margin);
        }
        clearance = fmin(clearance, fitting_distance(machine,nose_work) - machine->head_radius);
        clearance = fmin(clearance, fitting_distance(machine,housing_work) - machine->head_radius);
    }
    return clearance;
}

int sg_configuration_valid(const sg_machine *machine, const double *joint) {
    for (int axis = 0; axis < machine->count; ++axis)
        if (!isfinite(joint[axis]) || joint[axis] < machine->axes[axis].lower - 1e-10 ||
            joint[axis] > machine->axes[axis].upper + 1e-10) return 0;
    return sg_clearance(machine, joint) > 1e-7;
}

typedef struct {
    const sg_machine *machine;
    const sg_waypoint *first, *second;
    const double *start, *finish;
    double position_curvature, direction_curvature, clearance_speed;
    sg_diagnostic *diagnostic;
} edge_context;

static int edge_sample(const edge_context *context, double parameter,
                       double *position_error, double *direction_error, double *clearance) {
    double joint[SG_AXES] = {0};
    for (int axis = 0; axis < context->machine->count; ++axis)
        joint[axis] = context->start[axis] + parameter * (context->finish[axis] - context->start[axis]);
    sg_pose actual = sg_forward(context->machine, joint);
    sg_vector target_position = sg_add(context->first->pose.position,
        sg_scale(sg_subtract(context->second->pose.position, context->first->pose.position), parameter));
    sg_vector target_direction = sg_add(context->first->pose.direction,
        sg_scale(sg_subtract(context->second->pose.direction, context->first->pose.direction), parameter));
    target_direction = sg_scale(target_direction, 1 / sg_length(target_direction));
    *position_error = sg_length(sg_subtract(actual.position, target_position));
    *direction_error = sg_length(sg_subtract(actual.direction, target_direction));
    *clearance = sg_clearance(context->machine, joint);
    if (*position_error > context->machine->position_tolerance || *direction_error > context->machine->direction_tolerance) {
        sg_fail(context->diagnostic, SG_NO_CONTINUATION, 0, "joint interpolation leaves the specified tool-position or direction corridor");
        return 0;
    }
    if (*clearance <= 1e-7) {
        sg_fail(context->diagnostic, SG_NO_CONTINUATION, 0, "swept tool or housing collides with the modeled fitting or fixture");
        return 0;
    }
    return 1;
}

static int certify_interval(const edge_context *context, double lower, double upper,
                             double lower_position, double lower_direction,
                             double upper_position, double upper_direction, int depth) {
    double width = upper - lower, position_error, direction_error, clearance;
    if (!edge_sample(context, (lower+upper)/2, &position_error, &direction_error, &clearance)) return 0;
    /* Endpoint error + second derivative bound/8 bounds the whole error curve.
       Midpoint clearance - first derivative bound/2 bounds its swept clearance. */
    int geometry_proved = fmax(lower_position,upper_position) + context->position_curvature * width * width / 8 <= context->machine->position_tolerance &&
        fmax(lower_direction,upper_direction) + context->direction_curvature * width * width / 8 <= context->machine->direction_tolerance;
    int clearance_proved = clearance - context->clearance_speed * width / 2 > 1e-7;
    if (geometry_proved && clearance_proved) return 1;
    if (depth == 14) {
        sg_fail(context->diagnostic, SG_NO_CONTINUATION, 0, "interval bound inconclusive at subdivision limit; edge rejected");
        return 0;
    }
    return certify_interval(context, lower, (lower+upper)/2, lower_position,lower_direction,position_error,direction_error,depth+1) &&
        certify_interval(context,(lower+upper)/2,upper,position_error,direction_error,upper_position,upper_direction,depth+1);
}

int sg_edge_valid(const sg_machine *machine, const sg_waypoint *first,
                  const sg_waypoint *second, const double *start,
                  const double *finish, sg_diagnostic *diagnostic) {
    if (!sg_configuration_valid(machine, start) || !sg_configuration_valid(machine, finish))
        return !sg_fail(diagnostic, SG_NO_CONTINUATION, 0, "edge endpoint violates a joint limit or collision constraint");
    double direction_dot = sg_dot(first->pose.direction, second->pose.direction);
    if (direction_dot < -0.5) return !sg_fail(diagnostic, SG_NO_CONTINUATION, 0, "orientation step exceeds 120 degrees; supply an explicit interpolated path");
    double head_angle = fabs(finish[3]-start[3]) + fabs(finish[4]-start[4]);
    double table_angle = machine->count == 12 ? fabs(finish[10]-start[10])+fabs(finish[11]-start[11]) : 0;
    sg_vector nose_first = {start[0],start[1],start[2]}, nose_last = {finish[0],finish[1],finish[2]};
    sg_vector origin_first = {0}, origin_last = {0};
    if (machine->count == 12) {
        nose_first = sg_add(nose_first,(sg_vector){start[5],start[6],start[7]});
        nose_last = sg_add(nose_last,(sg_vector){finish[5],finish[6],finish[7]});
        origin_first = (sg_vector){start[8],start[9],0}; origin_last = (sg_vector){finish[8],finish[9],0};
    }
    double translation_speed = sg_length(sg_subtract(sg_subtract(nose_last,origin_last),sg_subtract(nose_first,origin_first)));
    double relative_size = fmax(sg_length(sg_subtract(nose_first,origin_first)),sg_length(sg_subtract(nose_last,origin_last))) + machine->tool_length;
    double relative_speed = translation_speed + machine->tool_length * head_angle + table_angle * relative_size;
    double total_angle = head_angle + table_angle;
    double direction_step = sg_length(sg_subtract(first->pose.direction,second->pose.direction));
    double minimum_direction_length = sqrt(fmax(0.25,(1+direction_dot)/2));
    edge_context context = {machine, first,second,start,finish,
        table_angle*table_angle*relative_size + 2*table_angle*(translation_speed+machine->tool_length*head_angle) + machine->tool_length*head_angle*head_angle,
        total_angle*total_angle + 6*direction_step*direction_step/pow(minimum_direction_length,3),
        fmax(relative_speed+(machine->tool_length+machine->housing_offset)*total_angle,
             sg_length(sg_subtract(nose_last,nose_first))+(machine->tool_length+machine->housing_offset)*head_angle), diagnostic};
    double first_position,first_direction,first_clearance,last_position,last_direction,last_clearance;
    if (!edge_sample(&context,0,&first_position,&first_direction,&first_clearance) ||
        !edge_sample(&context,1,&last_position,&last_direction,&last_clearance)) return 0;
    return certify_interval(&context,0,1,first_position,first_direction,last_position,last_direction,0);
}

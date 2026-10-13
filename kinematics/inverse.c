#include "gcode_planner.h"
#include <math.h>
#include <string.h>

/* Projection onto a bounded affine translation fiber. Minimize squared
   displacement from the declared initial joints, subject to the exact sum.
   An active-set projection handles limits instead of dividing blindly. */
static int allocate_linear(const sg_machine *machine, double *joint,
                           const int *indices, const double *signs,
                           int count, double requested) {
    int fixed[3] = {0};
    for (int entry = 0; entry < count; ++entry)
        joint[indices[entry]] = machine->axes[indices[entry]].initial;
    for (int iteration = 0; iteration <= count; ++iteration) {
        double current = 0;
        int free_count = 0;
        for (int entry = 0; entry < count; ++entry) {
            current += signs[entry] * joint[indices[entry]];
            free_count += !fixed[entry];
        }
        if (!free_count) return fabs(current - requested) < 1e-8;
        double correction = (requested - current) / free_count;
        int clipped = 0;
        for (int entry = 0; entry < count; ++entry) {
            int axis = indices[entry];
            if (fixed[entry]) continue;
            double next = joint[axis] + signs[entry] * correction;
            if (next < machine->axes[axis].lower || next > machine->axes[axis].upper) {
                joint[axis] = fmax(machine->axes[axis].lower, fmin(machine->axes[axis].upper, next));
                fixed[entry] = 1; clipped = 1;
            }
        }
        if (!clipped) {
            for (int entry = 0; entry < count; ++entry)
                if (!fixed[entry]) joint[indices[entry]] += signs[entry] * correction;
            return 1;
        }
    }
    return 0;
}

static int candidate_for_angles(const sg_machine *machine, sg_pose pose,
                                double tilt, double spin, double table_tilt,
                                double table_spin, sg_configuration *candidate) {
    memset(candidate, 0, sizeof *candidate);
    candidate->joint[3] = tilt; candidate->joint[4] = spin;
    candidate->branch = tilt < -1e-10 ? -1 : tilt > 1e-10 ? 1 : 0;
    sg_rotation table = sg_rotation_product(sg_rotate_z(table_spin), sg_rotate_x(table_tilt));
    sg_vector head_direction = sg_rotate(table, pose.direction);
    sg_vector required_nose = sg_add(sg_rotate(table, pose.position), sg_scale(head_direction, machine->tool_length));
    if (machine->count == 5) {
        candidate->joint[0] = required_nose.x;
        candidate->joint[1] = required_nose.y;
        candidate->joint[2] = required_nose.z;
    } else {
        const int x_indices[3] = {0,5,8}, y_indices[3] = {1,6,9}, z_indices[2] = {2,7};
        const double carriage_signs[3] = {1,1,-1}, z_signs[2] = {1,1};
        if (!allocate_linear(machine, candidate->joint, x_indices, carriage_signs, 3, required_nose.x) ||
            !allocate_linear(machine, candidate->joint, y_indices, carriage_signs, 3, required_nose.y) ||
            !allocate_linear(machine, candidate->joint, z_indices, z_signs, 2, required_nose.z)) return 0;
        candidate->joint[10] = table_tilt; candidate->joint[11] = table_spin;
    }
    if (!sg_configuration_valid(machine, candidate->joint)) return 0;
    sg_pose actual = sg_forward(machine, candidate->joint);
    return sg_length(sg_subtract(actual.position, pose.position)) < 1e-7 &&
        sg_length(sg_subtract(actual.direction, pose.direction)) < 1e-8;
}

static int append_candidate(const sg_machine *machine, sg_configuration *candidates,
                            int *count, const sg_configuration *candidate) {
    for (int existing = 0; existing < *count; ++existing) {
        double difference = 0;
        for (int axis = 0; axis < machine->count; ++axis)
            difference = fmax(difference, fabs(candidates[existing].joint[axis] - candidate->joint[axis]));
        if (difference < 1e-8) return 1;
    }
    if (*count == SG_CANDIDATES) return 0;
    candidates[(*count)++] = *candidate;
    return 1;
}

int sg_inverse_candidates(const sg_machine *machine, const sg_toolpath *path,
                          int point, sg_configuration *candidates,
                          sg_diagnostic *diagnostic) {
    int count = 0;
    if (point == 0) {
        sg_configuration initial = {{0}, 0};
        for (int axis = 0; axis < machine->count; ++axis) initial.joint[axis] = machine->axes[axis].initial;
        sg_pose actual = sg_forward(machine, initial.joint);
        if (sg_length(sg_subtract(actual.position, path->points[0].pose.position)) > 1e-7 ||
            sg_length(sg_subtract(actual.direction, path->points[0].pose.direction)) > 1e-8 ||
            !sg_configuration_valid(machine, initial.joint)) {
            sg_fail(diagnostic, SG_UNREACHABLE, 0, "first pose must match the feasible declared initial joints; no positioning move is invented");
            return -1;
        }
        initial.branch = initial.joint[3] < 0 ? -1 : initial.joint[3] > 0 ? 1 : 0;
        candidates[0] = initial;
        return 1;
    }
    double table_angles[3] = {-SG_PI/6, 0, SG_PI/6};
    int table_count = machine->count == 12 ? 3 : 1;
    for (int table_first = 0; table_first < table_count; ++table_first)
        for (int table_second = 0; table_second < table_count; ++table_second) {
            double table_tilt = machine->count == 12 ? table_angles[table_first] : 0;
            double table_spin = machine->count == 12 ? table_angles[table_second] : 0;
            sg_rotation table = sg_rotation_product(sg_rotate_z(table_spin), sg_rotate_x(table_tilt));
            sg_vector direction = sg_rotate(table, path->points[point].pose.direction);
            double tilt = atan2(hypot(direction.x, direction.y), direction.z);
            double spin_seeds[8];
            int seed_count = 0;
            if (hypot(direction.x, direction.y) > 1e-10) {
                spin_seeds[seed_count++] = atan2(direction.y, direction.x);
            } else {
                /* A pole has a continuous rotary fiber. Borrow the limiting
                   longitude on both sides, including across consecutive poles. */
                spin_seeds[seed_count++] = machine->axes[4].initial;
                for (int direction_sign = -1; direction_sign <= 1; direction_sign += 2)
                    for (int next = point + direction_sign; next >= 0 && next < path->count; next += direction_sign) {
                        sg_vector neighbour = sg_rotate(table, path->points[next].pose.direction);
                        if (hypot(neighbour.x, neighbour.y) <= 1e-10) continue;
                        spin_seeds[seed_count++] = atan2(neighbour.y, neighbour.x);
                        break;
                    }
            }
            for (int seed = 0; seed < seed_count; ++seed)
                for (int branch = 0; branch < 2; ++branch) {
                    double base_tilt = branch ? -tilt : tilt;
                    double base_spin = spin_seeds[seed] + (branch ? SG_PI : 0);
                    int first_tilt_wrap = (int)ceil((machine->axes[3].lower - base_tilt - 1e-10)/(2*SG_PI));
                    int last_tilt_wrap = (int)floor((machine->axes[3].upper - base_tilt + 1e-10)/(2*SG_PI));
                    int first_spin_wrap = (int)ceil((machine->axes[4].lower - base_spin - 1e-10)/(2*SG_PI));
                    int last_spin_wrap = (int)floor((machine->axes[4].upper - base_spin + 1e-10)/(2*SG_PI));
                    for (int tilt_wrap = first_tilt_wrap; tilt_wrap <= last_tilt_wrap; ++tilt_wrap)
                        for (int spin_wrap = first_spin_wrap; spin_wrap <= last_spin_wrap; ++spin_wrap) {
                            sg_configuration candidate;
                            if (candidate_for_angles(machine, path->points[point].pose,
                                base_tilt + 2*SG_PI*tilt_wrap, base_spin + 2*SG_PI*spin_wrap,
                                table_tilt, table_spin, &candidate) && !append_candidate(machine, candidates, &count, &candidate)) {
                                sg_fail(diagnostic, SG_CAPACITY, point, "inverse candidate capacity reached; no candidates silently discarded");
                                return -1;
                            }
                        }
                }
        }
    if (!count) {
        sg_fail(diagnostic, SG_UNREACHABLE, point, "no feasible configuration in the enumerated branches and redundant samples");
        return -1;
    }
    return count;
}

int sg_principal_baseline(const sg_machine *machine, const sg_toolpath *path,
                          sg_plan *plan, sg_diagnostic *diagnostic) {
    if (machine->count != 5) return sg_fail(diagnostic, SG_INPUT, 0, "principal baseline is defined only for head5");
    memset(plan, 0, sizeof *plan); plan->count = path->count;
    sg_configuration initial[SG_CANDIDATES];
    if (sg_inverse_candidates(machine, path, 0, initial, diagnostic) < 0) return diagnostic->code;
    plan->configuration[0] = initial[0];
    for (int point = 1; point < path->count; ++point) {
        sg_vector direction = path->points[point].pose.direction;
        double tilt = atan2(hypot(direction.x, direction.y), direction.z);
        double spin = hypot(direction.x, direction.y) < 1e-10 ? 0 : atan2(direction.y, direction.x);
        if (!candidate_for_angles(machine, path->points[point].pose, tilt, spin, 0, 0, &plan->configuration[point]))
            return sg_fail(diagnostic, SG_UNREACHABLE, point, "principal baseline endpoint violates an axis limit or collision constraint");
        plan->seconds[point] = sg_segment_seconds(machine, plan->configuration[point-1].joint,
            plan->configuration[point].joint, path->points[point].seconds);
    }
    return SG_OK;
}

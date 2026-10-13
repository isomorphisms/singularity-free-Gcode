#include "gcode_planner.h"
#include <math.h>
#include <string.h>

sg_vector sg_add(sg_vector first, sg_vector second) {
    return (sg_vector){first.x + second.x, first.y + second.y, first.z + second.z};
}
sg_vector sg_subtract(sg_vector first, sg_vector second) {
    return (sg_vector){first.x - second.x, first.y - second.y, first.z - second.z};
}
sg_vector sg_scale(sg_vector vector, double factor) {
    return (sg_vector){vector.x * factor, vector.y * factor, vector.z * factor};
}
double sg_dot(sg_vector first, sg_vector second) {
    return first.x * second.x + first.y * second.y + first.z * second.z;
}
double sg_length(sg_vector vector) { return sqrt(sg_dot(vector, vector)); }
sg_vector sg_rotate(sg_rotation rotation, sg_vector vector) {
    double coordinates[3] = {vector.x, vector.y, vector.z}, answer[3] = {0};
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
            answer[row] += rotation.entry[row][column] * coordinates[column];
    return (sg_vector){answer[0], answer[1], answer[2]};
}
sg_rotation sg_rotation_product(sg_rotation first, sg_rotation second) {
    sg_rotation product = {{{0}}};
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
            for (int middle = 0; middle < 3; ++middle)
                product.entry[row][column] += first.entry[row][middle] * second.entry[middle][column];
    return product;
}
sg_rotation sg_rotation_transpose(sg_rotation rotation) {
    sg_rotation result;
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
            result.entry[row][column] = rotation.entry[column][row];
    return result;
}
sg_rotation sg_rotate_x(double angle) {
    double cosine = cos(angle), sine = sin(angle);
    return (sg_rotation){{{1,0,0},{0,cosine,-sine},{0,sine,cosine}}};
}
sg_rotation sg_rotate_y(double angle) {
    double cosine = cos(angle), sine = sin(angle);
    return (sg_rotation){{{cosine,0,sine},{0,1,0},{-sine,0,cosine}}};
}
sg_rotation sg_rotate_z(double angle) {
    double cosine = cos(angle), sine = sin(angle);
    return (sg_rotation){{{cosine,-sine,0},{sine,cosine,0},{0,0,1}}};
}

/* One spindle. The twelve-joint extension adds an XYZ carriage, an XY
   workpiece carriage, and two table rotations, never another toolhead. */
sg_pose sg_forward(const sg_machine *machine, const double *joint) {
    sg_rotation head = sg_rotation_product(sg_rotate_z(joint[4]), sg_rotate_y(joint[3]));
    sg_rotation table = sg_rotate_x(0);
    sg_vector nose = {joint[0], joint[1], joint[2]}, work_origin = {0};
    if (machine->count == 12) {
        nose = sg_add(nose, (sg_vector){joint[5], joint[6], joint[7]});
        work_origin = (sg_vector){joint[8], joint[9], 0};
        table = sg_rotation_product(sg_rotate_z(joint[11]), sg_rotate_x(joint[10]));
    }
    sg_rotation inverse_table = sg_rotation_transpose(table);
    sg_vector head_direction = sg_rotate(head, (sg_vector){0,0,1});
    sg_vector tool_tip = sg_subtract(nose, sg_scale(head_direction, machine->tool_length));
    sg_pose pose;
    pose.rotation = sg_rotation_product(inverse_table, head);
    pose.direction = sg_rotate(pose.rotation, (sg_vector){0,0,1});
    pose.position = sg_rotate(inverse_table, sg_subtract(tool_tip, work_origin));
    return pose;
}

static sg_vector cross(sg_vector first, sg_vector second) {
    return (sg_vector){first.y*second.z-first.z*second.y,
        first.z*second.x-first.x*second.z, first.x*second.y-first.y*second.x};
}

/* A tangent basis avoids the identically zero sixth singular value of the
   redundant (position,nx,ny,nz) representation of position x S2. */
static void tangent_basis(sg_vector direction, sg_vector *first, sg_vector *second) {
    sg_vector reference = fabs(direction.z) < 0.8 ? (sg_vector){0,0,1} : (sg_vector){1,0,0};
    *first = cross(direction, reference);
    *first = sg_scale(*first, 1 / sg_length(*first));
    *second = cross(direction, *first);
}

void sg_jacobian(const sg_machine *machine, const double *joint,
                 int full_frame, double jacobian[6][SG_AXES]) {
    memset(jacobian, 0, 6 * SG_AXES * sizeof(double));
    sg_pose center = sg_forward(machine, joint);
    sg_vector first_tangent, second_tangent;
    tangent_basis(center.direction, &first_tangent, &second_tangent);
    for (int axis = 0; axis < machine->count; ++axis) {
        double plus[SG_AXES], minus[SG_AXES];
        memcpy(plus, joint, SG_AXES * sizeof(double));
        memcpy(minus, joint, SG_AXES * sizeof(double));
        double step = machine->axes[axis].rotary ? 1e-5 : 1e-3;
        plus[axis] += step; minus[axis] -= step;
        sg_pose forward = sg_forward(machine, plus), backward = sg_forward(machine, minus);
        double scale = machine->axes[axis].rotary ? 1 : machine->tool_length;
        sg_vector translation = sg_scale(sg_subtract(forward.position, backward.position),
                                        scale / (2 * step * machine->tool_length));
        jacobian[0][axis] = translation.x;
        jacobian[1][axis] = translation.y;
        jacobian[2][axis] = translation.z;
        if (!full_frame) {
            sg_vector direction_change = sg_scale(sg_subtract(forward.direction, backward.direction), scale / (2 * step));
            jacobian[3][axis] = sg_dot(direction_change, first_tangent);
            jacobian[4][axis] = sg_dot(direction_change, second_tangent);
        } else {
            double angular[3][3] = {{0}};
            for (int row = 0; row < 3; ++row)
                for (int column = 0; column < 3; ++column)
                    for (int middle = 0; middle < 3; ++middle)
                        angular[row][column] += (forward.rotation.entry[row][middle] - backward.rotation.entry[row][middle])
                            * center.rotation.entry[column][middle] * scale / (2 * step);
            jacobian[3][axis] = (angular[2][1] - angular[1][2]) / 2;
            jacobian[4][axis] = (angular[0][2] - angular[2][0]) / 2;
            jacobian[5][axis] = (angular[1][0] - angular[0][1]) / 2;
        }
    }
}

/* One-sided Jacobi SVD rotates columns of J transpose, without forming J*J^T
   and squaring its condition number. Its at most six columns are task rows. */
sg_condition sg_analyze(const sg_machine *machine, const double *joint,
                        int full_frame, int freeze_limits) {
    double jacobian[6][SG_AXES], matrix[SG_AXES][6] = {{0}};
    int rows = full_frame ? 6 : 5;
    sg_jacobian(machine, joint, full_frame, jacobian);
    for (int axis = 0; axis < machine->count; ++axis)
        for (int row = 0; row < rows; ++row) {
            int active = freeze_limits && (joint[axis] <= machine->axes[axis].lower + 1e-8 || joint[axis] >= machine->axes[axis].upper - 1e-8);
            matrix[axis][row] = active ? 0 : jacobian[row][axis];
        }
    for (int sweep = 0; sweep < 80; ++sweep) {
        int changed = 0;
        for (int first = 0; first < rows; ++first)
            for (int second = first + 1; second < rows; ++second) {
                double first_norm = 0, second_norm = 0, inner = 0;
                for (int axis = 0; axis < machine->count; ++axis) {
                    first_norm += matrix[axis][first] * matrix[axis][first];
                    second_norm += matrix[axis][second] * matrix[axis][second];
                    inner += matrix[axis][first] * matrix[axis][second];
                }
                if (fabs(inner) <= 1e-14 * sqrt(first_norm * second_norm) || fabs(inner) < 1e-30) continue;
                double ratio = (second_norm - first_norm) / (2 * inner);
                double tangent = copysign(1.0, ratio) / (fabs(ratio) + hypot(1.0, ratio));
                double cosine = 1 / hypot(1.0, tangent), sine = cosine * tangent;
                for (int axis = 0; axis < machine->count; ++axis) {
                    double first_entry = matrix[axis][first], second_entry = matrix[axis][second];
                    matrix[axis][first] = cosine * first_entry - sine * second_entry;
                    matrix[axis][second] = sine * first_entry + cosine * second_entry;
                }
                changed = 1;
            }
        if (!changed) break;
    }
    sg_condition condition = {1e100, 0, 0};
    double values[6];
    for (int row = 0; row < rows; ++row) {
        values[row] = 0;
        for (int axis = 0; axis < machine->count; ++axis)
            values[row] = hypot(values[row], matrix[axis][row]);
        condition.largest_singular_value = fmax(condition.largest_singular_value, values[row]);
    }
    for (int row = 0; row < rows; ++row) {
        condition.minimum_singular_value = fmin(condition.minimum_singular_value, values[row]);
        if (values[row] > condition.largest_singular_value * 1e-8) ++condition.rank;
    }
    return condition;
}

double sg_velocity_residual(const sg_machine *machine, const double *joint,
                            const double desired[5], double *velocity) {
    double jacobian[6][SG_AXES], residual[5], scaled[SG_AXES] = {0};
    sg_jacobian(machine, joint, 0, jacobian);
    for (int row = 0; row < 5; ++row) residual[row] = desired[row];
    /* Cyclic bounded coordinate descent solves a convex box-constrained LS
       problem. At a limit the box is one-sided, rather than deleting a column.
       A nonzero residual is reported; convergence is never taken as proof. */
    for (int iteration = 0; iteration < 2000; ++iteration) {
        double largest_change = 0;
        for (int axis = 0; axis < machine->count; ++axis) {
            double scale = machine->axes[axis].rotary ? 1 : machine->tool_length;
            double lower = -machine->axes[axis].velocity / scale;
            double upper = machine->axes[axis].velocity / scale;
            if (joint[axis] <= machine->axes[axis].lower + 1e-8) lower = 0;
            if (joint[axis] >= machine->axes[axis].upper - 1e-8) upper = 0;
            double numerator = 0, denominator = 1e-14;
            for (int row = 0; row < 5; ++row) {
                numerator += jacobian[row][axis] * residual[row];
                denominator += jacobian[row][axis] * jacobian[row][axis];
            }
            double next = fmax(lower, fmin(upper, scaled[axis] + numerator / denominator));
            double change = next - scaled[axis];
            for (int row = 0; row < 5; ++row) residual[row] -= jacobian[row][axis] * change;
            largest_change = fmax(largest_change, fabs(change));
            scaled[axis] = next;
        }
        if (largest_change < 1e-12) break;
    }
    double norm = 0;
    for (int row = 0; row < 5; ++row) norm = hypot(norm, residual[row]);
    for (int axis = 0; axis < machine->count; ++axis)
        velocity[axis] = scaled[axis] * (machine->axes[axis].rotary ? 1 : machine->tool_length);
    return norm;
}

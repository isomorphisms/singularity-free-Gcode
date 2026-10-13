#ifndef GCODE_PLANNER_H
#define GCODE_PLANNER_H

#include <stddef.h>
#include <stdio.h>

#define SG_AXES 12
#define SG_POINTS 1024
#define SG_CANDIDATES 160
#define SG_BEAM 32
#define SG_OBSTACLES 16
#define SG_PI 3.14159265358979323846

/* All internal rotary coordinates are radians; lengths are millimetres. */
typedef struct { double x, y, z; } sg_vector;
typedef struct { double entry[3][3]; } sg_rotation;
typedef struct { sg_vector position, direction; sg_rotation rotation; } sg_pose;
typedef struct {
    char name;
    int rotary;
    double lower, upper, velocity, acceleration, jerk, initial;
} sg_axis;
typedef struct { sg_vector center; double radius; } sg_sphere;
typedef struct {
    int count;
    sg_axis axes[SG_AXES]; /* X Y Z B C [U V W P Q A E] */
    double tool_length, tool_radius, head_radius, housing_offset;
    double position_tolerance, direction_tolerance, spindle_rpm;
    int obstacle_count;
    sg_sphere obstacles[SG_OBSTACLES]; /* fixed in machine coordinates */
    int fitting;
    sg_vector stock_half_size;
    double main_bore_radius, branch_bore_radius, branch_angle;
} sg_machine;
typedef struct {
    sg_pose pose;
    double seconds; /* nominal duration from preceding point */
    char object[64], operation[64];
    int cutting;
} sg_waypoint;
typedef struct { int count; sg_waypoint points[SG_POINTS]; } sg_toolpath;
typedef struct { double joint[SG_AXES]; int branch; } sg_configuration;
typedef struct {
    int count;
    sg_configuration configuration[SG_POINTS];
    double seconds[SG_POINTS];
    double cost;
} sg_plan;
typedef struct {
    int code, point;
    char message[240];
} sg_diagnostic;
typedef struct {
    double minimum_singular_value, largest_singular_value;
    int rank;
} sg_condition;
typedef struct {
    double travel[SG_AXES], maximum_velocity[SG_AXES];
    double maximum_acceleration[SG_AXES], maximum_jerk[SG_AXES];
    double minimum_singular_value, minimum_clearance;
    double maximum_position_error, maximum_direction_error;
    int rotary_reversals, large_rotary_steps, added_moves, invalid_edges;
} sg_metrics;

enum { SG_OK, SG_INPUT, SG_UNREACHABLE, SG_NO_CONTINUATION, SG_DIALECT,
       SG_IO, SG_CAPACITY };

sg_vector sg_add(sg_vector first, sg_vector second);
sg_vector sg_subtract(sg_vector first, sg_vector second);
sg_vector sg_scale(sg_vector vector, double factor);
double sg_dot(sg_vector first, sg_vector second);
double sg_length(sg_vector vector);
sg_vector sg_rotate(sg_rotation rotation, sg_vector vector);
sg_rotation sg_rotation_product(sg_rotation first, sg_rotation second);
sg_rotation sg_rotation_transpose(sg_rotation rotation);
sg_rotation sg_rotate_x(double angle);
sg_rotation sg_rotate_y(double angle);
sg_rotation sg_rotate_z(double angle);

void sg_default_machine(sg_machine *machine, int count);
int sg_read_machine(FILE *input, sg_machine *machine, sg_diagnostic *diagnostic);
int sg_read_toolpath(FILE *input, sg_toolpath *path, sg_diagnostic *diagnostic);
sg_pose sg_forward(const sg_machine *machine, const double *joint);
void sg_jacobian(const sg_machine *machine, const double *joint,
                 int full_frame, double jacobian[6][SG_AXES]);
sg_condition sg_analyze(const sg_machine *machine, const double *joint,
                        int full_frame, int freeze_limits);
/* Bounded velocity least squares. Columns use axis speed scales; desired is
   (translation / tool_length, tangent-direction) per second. */
double sg_velocity_residual(const sg_machine *machine, const double *joint,
                            const double desired[5], double *velocity);
int sg_inverse_candidates(const sg_machine *machine, const sg_toolpath *path,
                          int point, sg_configuration *candidates,
                          sg_diagnostic *diagnostic);
double sg_clearance(const sg_machine *machine, const double *joint);
int sg_configuration_valid(const sg_machine *machine, const double *joint);
int sg_edge_valid(const sg_machine *machine, const sg_waypoint *first,
                  const sg_waypoint *second, const double *start,
                  const double *finish, sg_diagnostic *diagnostic);
double sg_segment_seconds(const sg_machine *machine, const double *first,
                          const double *second, double nominal);
int sg_plan_path(const sg_machine *machine, const sg_toolpath *path,
                 sg_plan *plan, sg_diagnostic *diagnostic);
/* Baseline is deliberately a documented principal-angle reference, not a
   representation of every industrial product. It does not certify edges. */
int sg_principal_baseline(const sg_machine *machine, const sg_toolpath *path,
                          sg_plan *plan, sg_diagnostic *diagnostic);
int sg_validate_plan(const sg_machine *machine, const sg_toolpath *path,
                     const sg_plan *plan, sg_diagnostic *diagnostic);
sg_metrics sg_measure(const sg_machine *machine, const sg_toolpath *path,
                      const sg_plan *plan);
int sg_write_linuxcnc(FILE *output, const sg_machine *machine,
                      const sg_toolpath *path, const sg_plan *plan,
                      sg_diagnostic *diagnostic);
int sg_write_trace(FILE *output, const sg_machine *machine,
                   const sg_toolpath *path, const sg_plan *plan);
int sg_write_simulation(FILE *output, const sg_machine *machine,
                        const sg_toolpath *path, const sg_plan *plan);
int sg_write_configuration_plot(FILE *output, const sg_machine *machine,
                                const sg_plan *baseline, const sg_plan *planned);
void sg_write_metrics(FILE *output, const sg_machine *machine,
                      const char *name, const sg_metrics *metrics);
int sg_fail(sg_diagnostic *diagnostic, int code, int point, const char *message);

#endif

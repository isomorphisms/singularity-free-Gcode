#include "gcode_planner.h"
#include <math.h>
#include <string.h>

sg_metrics sg_measure(const sg_machine *machine, const sg_toolpath *path,
                      const sg_plan *plan) {
    sg_metrics metrics = {0};
    metrics.minimum_singular_value = 1e100;
    metrics.minimum_clearance = 1e100;
    double last_displacement[SG_AXES] = {0};
    for (int point = 0; point < plan->count; ++point) {
        const double *joint = plan->configuration[point].joint;
        metrics.minimum_singular_value = fmin(metrics.minimum_singular_value,sg_analyze(machine,joint,0,0).minimum_singular_value);
        metrics.minimum_clearance = fmin(metrics.minimum_clearance,sg_clearance(machine,joint));
        if (!point) continue;
        if (!sg_edge_valid(machine,&path->points[point-1],&path->points[point],plan->configuration[point-1].joint,joint,NULL)) ++metrics.invalid_edges;
        for (int axis = 0; axis < machine->count; ++axis) {
            double displacement = joint[axis]-plan->configuration[point-1].joint[axis];
            double seconds = plan->seconds[point];
            metrics.travel[axis] += fabs(displacement);
            metrics.maximum_velocity[axis] = fmax(metrics.maximum_velocity[axis],1.875*fabs(displacement)/seconds);
            metrics.maximum_acceleration[axis] = fmax(metrics.maximum_acceleration[axis],10*sqrt(3)/3*fabs(displacement)/(seconds*seconds));
            metrics.maximum_jerk[axis] = fmax(metrics.maximum_jerk[axis],60*fabs(displacement)/(seconds*seconds*seconds));
            if (machine->axes[axis].rotary) {
                if (fabs(displacement) > SG_PI/2) ++metrics.large_rotary_steps;
                if (displacement*last_displacement[axis] < -1e-12) ++metrics.rotary_reversals;
                if (fabs(displacement) > 1e-6) last_displacement[axis] = displacement;
            }
        }
        for (int sample = 0; sample <= 16; ++sample) {
            double parameter = sample/16.0, interpolated[SG_AXES] = {0};
            for (int axis = 0; axis < machine->count; ++axis)
                interpolated[axis] = plan->configuration[point-1].joint[axis]+parameter*(joint[axis]-plan->configuration[point-1].joint[axis]);
            sg_pose actual = sg_forward(machine,interpolated);
            sg_vector target = sg_add(path->points[point-1].pose.position,
                sg_scale(sg_subtract(path->points[point].pose.position,path->points[point-1].pose.position),parameter));
            sg_vector direction = sg_add(path->points[point-1].pose.direction,
                sg_scale(sg_subtract(path->points[point].pose.direction,path->points[point-1].pose.direction),parameter));
            direction = sg_scale(direction,1/sg_length(direction));
            metrics.maximum_position_error = fmax(metrics.maximum_position_error,sg_length(sg_subtract(actual.position,target)));
            metrics.maximum_direction_error = fmax(metrics.maximum_direction_error,sg_length(sg_subtract(actual.direction,direction)));
            metrics.minimum_clearance = fmin(metrics.minimum_clearance,sg_clearance(machine,interpolated));
        }
    }
    return metrics;
}

void sg_write_metrics(FILE *output, const sg_machine *machine,
                      const char *name, const sg_metrics *metrics) {
    fprintf(output,"%s\trotary_reversals\t%d\tcount\n",name,metrics->rotary_reversals);
    fprintf(output,"%s\tlarge_rotary_steps\t%d\tcount\n",name,metrics->large_rotary_steps);
    fprintf(output,"%s\tadded_moves\t%d\tcount\n",name,metrics->added_moves);
    fprintf(output,"%s\tinvalid_edges\t%d\tcount\n",name,metrics->invalid_edges);
    fprintf(output,"%s\tminimum_sigma\t%.12g\tdimensionless\n",name,metrics->minimum_singular_value);
    fprintf(output,"%s\tminimum_clearance\t%.12g\tmm-lower-bound\n",name,metrics->minimum_clearance);
    fprintf(output,"%s\tmaximum_position_error\t%.12g\tmm-sampled\n",name,metrics->maximum_position_error);
    fprintf(output,"%s\tmaximum_direction_error\t%.12g\tunit-vector-chord-sampled\n",name,metrics->maximum_direction_error);
    for (int axis = 0; axis < machine->count; ++axis) {
        double unit = machine->axes[axis].rotary ? 180/SG_PI : 1;
        const char *unit_name = machine->axes[axis].rotary ? "degree" : "mm";
        fprintf(output,"%s\t%c_travel\t%.12g\t%s\n",name,machine->axes[axis].name,metrics->travel[axis]*unit,unit_name);
        fprintf(output,"%s\t%c_maximum_velocity\t%.12g\t%s/s-reference-quintic\n",name,machine->axes[axis].name,metrics->maximum_velocity[axis]*unit,unit_name);
        fprintf(output,"%s\t%c_maximum_acceleration\t%.12g\t%s/s2-reference-quintic\n",name,machine->axes[axis].name,metrics->maximum_acceleration[axis]*unit,unit_name);
        fprintf(output,"%s\t%c_maximum_jerk\t%.12g\t%s/s3-reference-quintic\n",name,machine->axes[axis].name,metrics->maximum_jerk[axis]*unit,unit_name);
    }
}

int sg_write_simulation(FILE *output, const sg_machine *machine,
                        const sg_toolpath *path, const sg_plan *plan) {
    fprintf(output,"segment\tobject\toperation\ttime_s\tpath_parameter\ttip_x_mm\ttip_y_mm\ttip_z_mm\tnx\tny\tnz\tclearance_lower_bound_mm");
    for (int axis = 0; axis < machine->count; ++axis) fprintf(output,"\t%c\t%c_velocity\t%c_acceleration",machine->axes[axis].name,machine->axes[axis].name,machine->axes[axis].name);
    fprintf(output,"\n");
    double elapsed = 0;
    for (int point = 1; point < plan->count; ++point) {
        double duration = plan->seconds[point];
        for (int sample = 0; sample <= 8; ++sample) {
            double time = sample/8.0;
            double parameter = time*time*time*(10+time*(-15+6*time));
            double speed = 30*time*time*(1-time)*(1-time)/duration;
            double acceleration = 60*time*(1-time)*(1-2*time)/(duration*duration);
            double joint[SG_AXES] = {0};
            for (int axis = 0; axis < machine->count; ++axis)
                joint[axis] = plan->configuration[point-1].joint[axis]+parameter*(plan->configuration[point].joint[axis]-plan->configuration[point-1].joint[axis]);
            sg_pose pose = sg_forward(machine,joint);
            fprintf(output,"%d\t%s\t%s\t%.12g\t%.12g\t%.12g\t%.12g\t%.12g\t%.12g\t%.12g\t%.12g\t%.12g",point,path->points[point].object,path->points[point].operation,
                elapsed+time*duration,parameter,pose.position.x,pose.position.y,pose.position.z,pose.direction.x,pose.direction.y,pose.direction.z,sg_clearance(machine,joint));
            for (int axis = 0; axis < machine->count; ++axis) {
                double displacement = plan->configuration[point].joint[axis]-plan->configuration[point-1].joint[axis];
                fprintf(output,"\t%.12g\t%.12g\t%.12g",joint[axis],speed*displacement,acceleration*displacement);
            }
            fprintf(output,"\n");
        }
        elapsed += duration;
    }
    return ferror(output) ? SG_IO : SG_OK;
}

int sg_write_configuration_plot(FILE *output, const sg_machine *machine,
                                const sg_plan *baseline, const sg_plan *planned) {
    (void)machine;
    double lower = -10, upper = 10;
    for (int point = 0; point < planned->count; ++point) {
        lower = fmin(lower,fmin(planned->configuration[point].joint[4],baseline->configuration[point].joint[4])*180/SG_PI);
        upper = fmax(upper,fmax(planned->configuration[point].joint[4],baseline->configuration[point].joint[4])*180/SG_PI);
    }
    fprintf(output,"<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"960\" height=\"780\" viewBox=\"0 0 960 780\">\n");
    fprintf(output,"<rect width=\"960\" height=\"780\" fill=\"white\"/><g font-family=\"sans-serif\" font-size=\"16\" fill=\"#222\"><text x=\"50\" y=\"28\">C rotary coordinate along toolpath (degrees)</text><text x=\"50\" y=\"410\">Waypoint index; orange: principal baseline, blue: continuous plan</text><text x=\"50\" y=\"455\">Configuration-space projection: B versus C (degrees)</text></g>\n");
    for (int tick = 0; tick <= 4; ++tick) {
        double value = lower+(upper-lower)*tick/4;
        fprintf(output,"<line x1=\"65\" y1=\"%.2f\" x2=\"930\" y2=\"%.2f\" stroke=\"#ddd\"/><text x=\"5\" y=\"%.2f\" font-size=\"13\">%.1f</text>\n",370-300.0*tick/4,370-300.0*tick/4,374-300.0*tick/4,value);
    }
    for (int series = 0; series < 2; ++series) {
        const sg_plan *plan = series ? planned : baseline;
        fprintf(output,"<polyline fill=\"none\" stroke=\"%s\" stroke-width=\"2.5\" points=\"",series?"#176b9b":"#d16c25");
        for (int point = 0; point < plan->count; ++point) {
            double value = plan->configuration[point].joint[4]*180/SG_PI;
            fprintf(output,"%.2f,%.2f ",65+865.0*point/(plan->count-1),370-300*(value-lower)/(upper-lower));
        }
        fprintf(output,"\"/>\n");
    }
    double minimum_tilt = -5, maximum_tilt = 5;
    for (int point = 0; point < planned->count; ++point) {
        minimum_tilt = fmin(minimum_tilt,fmin(planned->configuration[point].joint[3],baseline->configuration[point].joint[3])*180/SG_PI);
        maximum_tilt = fmax(maximum_tilt,fmax(planned->configuration[point].joint[3],baseline->configuration[point].joint[3])*180/SG_PI);
    }
    double pole_y = 720-240*(0-minimum_tilt)/(maximum_tilt-minimum_tilt);
    fprintf(output,"<line x1=\"65\" y1=\"%.2f\" x2=\"930\" y2=\"%.2f\" stroke=\"#b52b49\" stroke-dasharray=\"5 5\"/><text x=\"5\" y=\"%.2f\" font-size=\"12\">B=0</text>\n",pole_y,pole_y,pole_y-5);
    for (int series = 0; series < 2; ++series) {
        const sg_plan *plan = series ? planned : baseline;
        fprintf(output,"<polyline fill=\"none\" stroke=\"%s\" stroke-width=\"2.5\" points=\"",series?"#176b9b":"#d16c25");
        for (int point = 0; point < plan->count; ++point)
            fprintf(output,"%.2f,%.2f ",65+865*(plan->configuration[point].joint[4]*180/SG_PI-lower)/(upper-lower),
                720-240*(plan->configuration[point].joint[3]*180/SG_PI-minimum_tilt)/(maximum_tilt-minimum_tilt));
        fprintf(output,"\"/>\n");
    }
    fprintf(output,"<g font-family=\"sans-serif\" font-size=\"14\"><text x=\"65\" y=\"747\">C %.1f</text><text x=\"870\" y=\"747\">C %.1f</text><text x=\"5\" y=\"482\">B %.1f</text><text x=\"5\" y=\"720\">B %.1f</text><text x=\"65\" y=\"775\">The B=0 task pole can be crossed on a continuous signed-B branch.</text></g></svg>\n",lower,upper,maximum_tilt,minimum_tilt);
    return ferror(output) ? SG_IO : SG_OK;
}

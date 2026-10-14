#include "gcode_planner.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static int failures;
static void require(int condition, const char *message) {
    if (!condition) { fprintf(stderr,"FAIL %s\n",message); ++failures; }
}
static void write_machine(const char *filename, const sg_machine *machine) {
    FILE *output = fopen(filename,"w");
    if (!output) { require(0,filename); return; }
    fprintf(output,"# Exact geometry of the simulation reference; not a calibrated real mill.\n");
    fprintf(output,"machine %s mm-radians-v1\n",machine->count==5?"head5":"hybrid12");
    for (int axis = 0; axis < machine->count; ++axis) {
        sg_axis definition = machine->axes[axis];
        fprintf(output,"axis %c %s %.17g %.17g %.17g %.17g %.17g %.17g\n",definition.name,definition.rotary?"rotary":"linear",
            definition.lower,definition.upper,definition.velocity,definition.acceleration,definition.jerk,definition.initial);
    }
    fprintf(output,"tool %.17g %.17g %.17g %.17g\ntolerance %.17g %.17g\nspindle %.17g\n",machine->tool_length,machine->tool_radius,machine->head_radius,machine->housing_offset,
        machine->position_tolerance,machine->direction_tolerance,machine->spindle_rpm);
    if (machine->fitting) fprintf(output,"fitting %.17g %.17g %.17g %.17g %.17g %.17g\n",machine->stock_half_size.x,machine->stock_half_size.y,machine->stock_half_size.z,
        machine->main_bore_radius,machine->branch_bore_radius,machine->branch_angle);
    for (int obstacle = 0; obstacle < machine->obstacle_count; ++obstacle) {
        sg_sphere sphere = machine->obstacles[obstacle];
        fprintf(output,"sphere %.17g %.17g %.17g %.17g\n",sphere.center.x,sphere.center.y,sphere.center.z,sphere.radius);
    }
    require(!fclose(output),"machine write");
}
static void add_point(sg_toolpath *path, const char *object, const char *operation,
                      int cutting, sg_vector position, double tilt, double spin, double seconds) {
    if (path->count == SG_POINTS) { require(0,"fixture capacity"); return; }
    sg_waypoint *waypoint = &path->points[path->count++];
    memset(waypoint,0,sizeof *waypoint);
    snprintf(waypoint->object,sizeof waypoint->object,"%s",object);
    snprintf(waypoint->operation,sizeof waypoint->operation,"%s",operation);
    waypoint->cutting = cutting; waypoint->seconds = seconds; waypoint->pose.position = position;
    waypoint->pose.direction = (sg_vector){sin(tilt)*cos(spin),sin(tilt)*sin(spin),cos(tilt)};
}
static void move_line(sg_toolpath *path, const char *operation, int cutting,
                      sg_vector end, double tilt, int subdivisions) {
    sg_vector start = path->points[path->count-1].pose.position;
    for (int step = 1; step <= subdivisions; ++step)
        add_point(path,"tee-fitting",operation,cutting,sg_add(start,sg_scale(sg_subtract(end,start),(double)step/subdivisions)),tilt,0,0.4);
}
static void rotate_at_clearance(sg_toolpath *path, double first, double second) {
    sg_vector position = path->points[path->count-1].pose.position;
    int subdivisions = (int)ceil(fabs(second-first)/0.015);
    for (int step = 1; step <= subdivisions; ++step)
        add_point(path,"tee-fitting","orientation-at-clearance",0,position,first+(second-first)*step/subdivisions,0,0.3);
}
static void fitting_path(sg_toolpath *path) {
    path->count = 0;
    sg_vector clearance = {0,0,50};
    double branch_angle = SG_PI/18;
    add_point(path,"tee-fitting","initial",0,clearance,0,0,1);
    rotate_at_clearance(path,0,branch_angle);
    sg_vector branch_approach = {30*sin(branch_angle),0,30*cos(branch_angle)};
    move_line(path,"angled-port-approach",0,branch_approach,branch_angle,8);
    move_line(path,"angled-port-bore",1,(sg_vector){0,0,0},branch_angle,12);
    move_line(path,"angled-port-retract",0,branch_approach,branch_angle,12);
    move_line(path,"clearance-return",0,clearance,branch_angle,8);
    rotate_at_clearance(path,branch_angle,SG_PI/2);
    move_line(path,"main-bore-approach",0,(sg_vector){28,0,50},SG_PI/2,8);
    move_line(path,"main-bore-approach",0,(sg_vector){28,0,0},SG_PI/2,12);
    move_line(path,"intersecting-through-bore",1,(sg_vector){-24,0,0},SG_PI/2,20);
    move_line(path,"main-bore-retract",0,(sg_vector){28,0,0},SG_PI/2,20);
    move_line(path,"clearance-return",0,(sg_vector){28,0,50},SG_PI/2,12);
    move_line(path,"clearance-return",0,clearance,SG_PI/2,8);
    rotate_at_clearance(path,SG_PI/2,0.06);
    move_line(path,"junction-chamfer-approach",0,(sg_vector){3.5,0,2},0.06,16);
    for (int step = 1; step <= 80; ++step) {
        double angle = 2*SG_PI*step/80;
        double tilt = fabs(cos(angle)) < 1e-12 ? 0 : 0.06*cos(angle);
        add_point(path,"tee-fitting","internal-junction-chamfer",1,(sg_vector){3.5*cos(angle),3.5*sin(angle),2},tilt,0,0.4);
    }
    move_line(path,"junction-chamfer-retract",0,(sg_vector){3.5,0,50},0.06,16);
    move_line(path,"clearance-return",0,clearance,0.06,4);
    rotate_at_clearance(path,0.06,0);
}
static void twisted_path(sg_toolpath *path) {
    path->count = 0;
    for (int step = 0; step <= 80; ++step) {
        double parameter = step/80.0;
        double spin = (170+40*parameter)*SG_PI/180;
        double tilt = 0.4+0.03*sin(parameter*2*SG_PI);
        sg_vector position = {10*cos(parameter*SG_PI),10*sin(parameter*SG_PI),50+4*parameter};
        add_point(path,"twisted-surface","simultaneous-finish",step!=0,position,tilt,spin,0.4);
    }
}
static void write_toolpath(const char *filename, const sg_toolpath *path) {
    FILE *output = fopen(filename,"w");
    if (!output) { require(0,filename); return; }
    fprintf(output,"toolpath mm-unit-vector-v1\n# object operation cut|travel nominal_seconds x y z nx ny nz\n");
    for (int point = 0; point < path->count; ++point) {
        sg_waypoint waypoint = path->points[point];
        fprintf(output,"%s %s %s %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n",waypoint.object,waypoint.operation,waypoint.cutting?"cut":"travel",waypoint.seconds,
            waypoint.pose.position.x,waypoint.pose.position.y,waypoint.pose.position.z,waypoint.pose.direction.x,waypoint.pose.direction.y,waypoint.pose.direction.z);
    }
    require(!fclose(output),"toolpath write");
}
static void baseline_listing(FILE *output, const sg_machine *machine,
                              const sg_toolpath *path, const sg_plan *plan) {
    /* Keep the failed baseline's G-code inspectable, but terminate before its
       diagnostic motion listing. Each listing line is a comment, not motion. */
    fprintf(output,"(FAILED PRINCIPAL BASELINE - diagnostic G-code listing only)\nM2\n");
    fprintf(output,"(G21 G90 G17 G40 G49 G80 G54 G61 G93)\n");
    for (int point = 1; point < plan->count; ++point) {
        const double *joint = plan->configuration[point].joint;
        fprintf(output,"(source %d %s %s)\n",point,path->points[point].object,path->points[point].operation);
        fprintf(output,"(N%d G1 X%.10f Y%.10f Z%.10f B%.10f C%.10f F%.10f)\n",point*10,joint[0],joint[1],joint[2],joint[3]*180/SG_PI,joint[4]*180/SG_PI,60/plan->seconds[point]);
    }
    (void)machine;
}

static void evidence(const char *directory, sg_machine *machine, sg_toolpath *path) {
    char filename[256]; sg_diagnostic diagnostic = {0};
    sg_plan *baseline = malloc(sizeof *baseline), *plan = malloc(sizeof *plan);
    require(baseline && plan,"evidence allocation");
    if (!baseline || !plan) { free(baseline); free(plan); return; }
    snprintf(filename,sizeof filename,"%s/machine.sg",directory); write_machine(filename,machine);
    snprintf(filename,sizeof filename,"%s/toolpath.sg",directory); write_toolpath(filename,path);
    int result = sg_principal_baseline(machine,path,baseline,&diagnostic);
    require(!result,diagnostic.message);
    if (result) goto finish;
    result = sg_plan_path(machine,path,plan,&diagnostic);
    if (result) fprintf(stderr,"point %d: %s\n",diagnostic.point,diagnostic.message);
    require(!result,"example continuous planning");
    if (result) goto finish;
    sg_metrics before = sg_measure(machine,path,baseline), after = sg_measure(machine,path,plan);
    require(before.invalid_edges > 0,"baseline must demonstrate a real interpolation deficiency");
    require(after.invalid_edges == 0,"all planned edges certified");
    require(after.travel[4] < before.travel[4],"reduce C axis travel");
    require(after.added_moves == 0,"no generated positioning moves");
    snprintf(filename,sizeof filename,"%s/comparison.tsv",directory);
    FILE *output = fopen(filename,"w"); require(output!=NULL,filename);
    if (output) { fprintf(output,"planner\tmetric\tvalue\tunit\n"); sg_write_metrics(output,machine,"principal",&before); sg_write_metrics(output,machine,"continuous",&after); require(!fclose(output),filename); }
    snprintf(filename,sizeof filename,"%s/continuous.ngc",directory);
    output = fopen(filename,"w"); require(output!=NULL,filename);
    if (output) { require(!sg_write_linuxcnc(output,machine,path,plan,&diagnostic),diagnostic.message); require(!fclose(output),filename); }
    snprintf(filename,sizeof filename,"%s/baseline-diagnostic.ngc",directory);
    output = fopen(filename,"w"); require(output!=NULL,filename);
    if (output) { baseline_listing(output,machine,path,baseline); require(!fclose(output),filename); }
    snprintf(filename,sizeof filename,"%s/axis-trace.tsv",directory);
    output = fopen(filename,"w"); require(output!=NULL,filename);
    if (output) { require(!sg_write_trace(output,machine,path,plan),filename); require(!fclose(output),filename); }
    snprintf(filename,sizeof filename,"%s/baseline-trace.tsv",directory);
    output = fopen(filename,"w"); require(output!=NULL,filename);
    if (output) { require(!sg_write_trace(output,machine,path,baseline),filename); require(!fclose(output),filename); }
    snprintf(filename,sizeof filename,"%s/simulation.tsv",directory);
    output = fopen(filename,"w"); require(output!=NULL,filename);
    if (output) { require(!sg_write_simulation(output,machine,path,plan),filename); require(!fclose(output),filename); }
    snprintf(filename,sizeof filename,"%s/configuration-space.svg",directory);
    output = fopen(filename,"w"); require(output!=NULL,filename);
    if (output) { require(!sg_write_configuration_plot(output,machine,baseline,plan),filename); require(!fclose(output),filename); }
    printf("PASS %s: %d poses; baseline invalid edges %d -> %d; C travel %.6f -> %.6f degrees\n",directory,path->count,before.invalid_edges,after.invalid_edges,before.travel[4]*180/SG_PI,after.travel[4]*180/SG_PI);
finish:
    free(baseline); free(plan);
}

int main(void) {
    sg_machine machine; sg_toolpath *path = malloc(sizeof *path);
    if (!path) return SG_CAPACITY;
    sg_default_machine(&machine,5); machine.fitting = 1;
    machine.stock_half_size = (sg_vector){20,14,12};
    machine.main_bore_radius = 6; machine.branch_bore_radius = 8; machine.branch_angle = SG_PI/18;
    fitting_path(path); evidence("examples/pneumatic-fitting",&machine,path);
    sg_default_machine(&machine,5); twisted_path(path);
    sg_vector direction = path->points[0].pose.direction, position = path->points[0].pose.position;
    machine.axes[0].initial = position.x+machine.tool_length*direction.x;
    machine.axes[1].initial = position.y+machine.tool_length*direction.y;
    machine.axes[2].initial = position.z+machine.tool_length*direction.z;
    machine.axes[3].initial = 0.4; machine.axes[4].initial = 170*SG_PI/180;
    evidence("examples/twisted-surface",&machine,path);
    sg_default_machine(&machine,12);
    machine.axes[0].lower = -10; machine.axes[0].upper = 10;
    path->count = 0;
    add_point(path,"redundant-carriage","initial",0,(sg_vector){0,0,50},0,0,1);
    for (int step = 1; step <= 20; ++step)
        add_point(path,"redundant-carriage","limited-primary-X",0,(sg_vector){2.0*step,0,50},0,0,0.5);
    write_machine("machines/hybrid12.sg",&machine);
    write_toolpath("examples/twelve-axis/toolpath.sg",path);
    sg_plan *plan = malloc(sizeof *plan); sg_diagnostic diagnostic = {0};
    require(plan!=NULL,"twelve-joint plan allocation");
    if (plan) {
        int result = sg_plan_path(&machine,path,plan,&diagnostic);
        if (result) fprintf(stderr,"twelve point %d: %s\n",diagnostic.point,diagnostic.message);
        require(!result,"twelve-joint redundant allocation");
        if (!result) {
            FILE *output = fopen("examples/twelve-axis/axis-trace.tsv","w"); require(output!=NULL,"twelve trace");
            if (output) { require(!sg_write_trace(output,&machine,path,plan),"twelve trace write"); require(!fclose(output),"twelve trace close"); }
            require(fabs(plan->configuration[plan->count-1].joint[0]) <= 10+1e-8,"primary X limit");
            require(fabs(plan->configuration[plan->count-1].joint[5]) > 1,"secondary carriage used");
            printf("PASS twelve joints: X %.6f U %.6f P %.6f reach tool X=40 with primary X limited to +/-10\n",plan->configuration[plan->count-1].joint[0],plan->configuration[plan->count-1].joint[5],plan->configuration[plan->count-1].joint[8]);
        }
        free(plan);
    }
    free(path);
    return failures ? 1 : 0;
}

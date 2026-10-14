/* Exact two-dimensional quotient-ring arithmetic plus numerical annulus
   area measurements. This research executable is not linked into the post. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct { double constant, coordinate; } quotient_element;
static quotient_element multiply(quotient_element first, quotient_element second, double parameter) {
    return (quotient_element){first.constant*second.constant+parameter*first.coordinate*second.coordinate,
        first.constant*second.coordinate+first.coordinate*second.constant};
}
static double annuli_area(double separation, double epsilon) {
    double inner_squared = (1-epsilon)*(1-epsilon), outer_squared = (1+epsilon)*(1+epsilon);
    double lower_x = fmax(separation-(1+epsilon),separation/2-2*epsilon/separation);
    double upper_x = fmin(1+epsilon,separation/2+2*epsilon/separation);
    const int cells = 40000;
    double area = 0;
    for (int cell = 0; cell < cells; ++cell) {
        double coordinate_x = lower_x+(upper_x-lower_x)*(cell+0.5)/cells;
        double first_squared = coordinate_x*coordinate_x;
        double second_squared = (coordinate_x-separation)*(coordinate_x-separation);
        double lower_y_squared = fmax(0,fmax(inner_squared-first_squared,inner_squared-second_squared));
        double upper_y_squared = fmin(outer_squared-first_squared,outer_squared-second_squared);
        if (upper_y_squared > lower_y_squared)
            area += 2*(sqrt(upper_y_squared)-sqrt(lower_y_squared))*(upper_x-lower_x)/cells;
    }
    return area;
}

int main(int count, char **arguments) {
    FILE *output = stdout;
    if (count==2) output = fopen(arguments[1],"w");
    if (!output || count>2) return 1;
    int failures = 0;
    quotient_element coordinate = {0,1}, squared = multiply(coordinate,coordinate,0);
    failures += squared.constant!=0 || squared.coordinate!=0 || coordinate.coordinate==0;
    fprintf(output,"experiment\tparameter\treal_configurations\tmultiplicity\tjacobian_magnitude\tarea\tscaling_exponent\n");
    double parameters[] = {-1,-1e-8,0,1e-8,0.01,1};
    for (size_t experiment = 0; experiment < sizeof parameters/sizeof *parameters; ++experiment) {
        double parameter = parameters[experiment];
        int conventional_count = parameter<0?0:parameter==0?1:2;
        quotient_element product = multiply(coordinate,coordinate,parameter);
        failures += product.constant!=parameter || product.coordinate!=0;
        /* Polynomial discriminant and scheme support give identical real-root
           decisions in this family. Constant rank-two quotient basis retains
           algebraic length even when no real configurations exist. */
        double discriminant = 4*parameter;
        int discriminant_count = discriminant<0?0:discriminant==0?1:2;
        failures += conventional_count!=discriminant_count;
        fprintf(output,"x-squared-equals-t\t%.12g\t%d\t%d\t%.12g\tNA\tNA\n",parameter,conventional_count,parameter==0?2:1,parameter>0?2*sqrt(parameter):0);
    }
    for (int model = 0; model < 2; ++model) {
        double separation = model?2:1, previous_area = 0, previous_epsilon = 0;
        for (int step = 0; step < 5; ++step) {
            double epsilon = 0.02/pow(2,step), area = annuli_area(separation,epsilon);
            double exponent = step?log(area/previous_area)/log(epsilon/previous_epsilon):0;
            fprintf(output,"two-rods-d-%.0f\t%.12g\t%d\t%d\t%.12g\t%.12g\t%.12g\n",separation,epsilon,model?1:2,model?2:1,
                model?0:4*separation*sqrt(1-separation*separation/4),area,exponent);
            if (step==4) failures += fabs(exponent-(model?1.5:2))>0.03;
            previous_area = area; previous_epsilon = epsilon;
        }
    }
    fprintf(stderr,"%s quotient arithmetic, identical root decisions, annulus scaling 2 versus 3/2; scheme production benefit NOT_DEMONSTRATED\n",failures?"FAIL":"PASS");
    if (output!=stdout && fclose(output)) ++failures;
    return failures?1:0;
}

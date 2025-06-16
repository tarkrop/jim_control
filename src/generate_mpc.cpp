#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <acado_code_generation.hpp>

using namespace std;

USING_NAMESPACE_ACADO

int main( )
{

        // INTRODUCE THE VARIABLES (acadoVariables.x):
        // -------------------------
        DifferentialState x; 
        DifferentialState y;
        DifferentialState q;
        DifferentialState v;
        DifferentialState w;

        Control a;
        Control j;

        OnlineData obs1_x;
        OnlineData obs1_y;
        OnlineData obs1_radius;

        OnlineData obs2_x;
        OnlineData obs2_y;
        OnlineData obs2_radius;

        OnlineData obs3_x;
        OnlineData obs3_y;
        OnlineData obs3_radius;

        OnlineData obs4_x;
        OnlineData obs4_y;
        OnlineData obs4_radius;

        OnlineData obs5_x;
        OnlineData obs5_y;
        OnlineData obs5_radius;

        double L = 0.55;
        double dt = 0.1;
        double v_limit = 0.25;
        double w_limit = 1.0;
        double accel_limit = 3.0;
        double angular_accel_limit = 10.0;

        double eps = 1e-4;

        DifferentialEquation f;

        f << dot(x) == v*cos(q);
        f << dot(y) == v*sin(q);
        f << dot(q) == w;
        f << dot(v) == a;
        f << dot(w) == j;
        
        Expression d1 = sqrt(pow(x-obs1_x, 2) + pow(y-obs1_y, 2)) / (obs1_radius + eps);
        Expression d2 = sqrt(pow(x-obs2_x, 2) + pow(y-obs2_y, 2)) / (obs2_radius + eps);
        Expression d3 = sqrt(pow(x-obs3_x, 2) + pow(y-obs3_y, 2)) / (obs3_radius + eps);
        Expression d4 = sqrt(pow(x-obs4_x, 2) + pow(y-obs4_y, 2)) / (obs4_radius + eps);
        Expression d5 = sqrt(pow(x-obs5_x, 2) + pow(y-obs5_y, 2)) / (obs5_radius + eps);

        Function rf;
        Function rfN;

        rf << x << y << q << v << w << a << j;
        // rf << (1 / (1.0 + exp(-1 + sqrt((pow(x-obs1_x, 2) + pow(y-obs1_y, 2))/pow(obs1_radius+eps, 2)))) +
        //         1 / (1.0 + exp(-1 + sqrt((pow(x-obs2_x, 2) + pow(y-obs2_y, 2))/pow(obs2_radius+eps, 2)))) +
        //         1 / (1.0 + exp(-1 + sqrt((pow(x-obs3_x, 2) + pow(y-obs3_y, 2))/pow(obs3_radius+eps, 2)))) +
        //         1 / (1.0 + exp(-1 + sqrt((pow(x-obs4_x, 2) + pow(y-obs4_y, 2))/pow(obs4_radius+eps, 2)))) +
        //         1 / (1.0 + exp(-1 + sqrt((pow(x-obs5_x, 2) + pow(y-obs5_y, 2))/pow(obs5_radius+eps, 2)))));
        rf << (1/d1 + 1/d2 + 1/d3 + 1/d4 + 1/d5);

        rfN << x << y << q;

        const int N  = 50;
        const int Ni = 9;
        const double Ts = 0.1;


        BMatrix W = eye<bool>(rf.getDim());
        BMatrix WN = eye<bool>(rfN.getDim());
        
        OCP ocp(0, N * Ts, N);

        ocp.subjectTo( f );

        ocp.subjectTo( -v_limit <= v <= v_limit );
        ocp.subjectTo( -w_limit <= w <= w_limit );
        ocp.subjectTo( v + (w * L / 2) <= v_limit );
         ocp.subjectTo( -v_limit <= -v + (w * L / 2) );
        ocp.subjectTo( -accel_limit <= a <= accel_limit );
        ocp.subjectTo( -angular_accel_limit <= j <= angular_accel_limit );

        ocp.minimizeLSQ(W, rf);
        ocp.minimizeLSQEndTerm(WN, rfN);
        ocp.setNOD(15);


        OCPexport mpc( ocp );
        
        mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
        //mpc.set(DISCRETIZATION_TYPE, SINGLE_SHOOTING);        
        mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
        mpc.set(INTEGRATOR_TYPE, INT_RK45);
        // mpc.set(INTEGRATOR_TYPE, INT_IRK_RIIA3);
        mpc.set(NUM_INTEGRATOR_STEPS, N * Ni);
        mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
        //	mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
        mpc.set(QP_SOLVER, QP_QPOASES);
        //	mpc.set(QP_SOLVER, QP_FORCES);
        mpc.set(MAX_NUM_QP_ITERATIONS, 999);
        mpc.set(HOTSTART_QP, YES);        
        //	mpc.set(SPARSE_QP_SOLUTION, SPARSE_SOLVER);        
	mpc.set(LEVENBERG_MARQUARDT, 1.0e-4);
        mpc.set(GENERATE_TEST_FILE, YES);
        mpc.set(GENERATE_MAKE_FILE, YES);
        mpc.set(GENERATE_MATLAB_INTERFACE, YES);
        //	mpc.set(USE_SINGLE_PRECISION, YES);
        mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);
        mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO);

        //	mpc.set(CG_USE_OPENMP, YES);
        // NOTE: This is crucial for export of MHE!
	      //mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
	//       mpc.set(FIX_INITIAL_STATE, YES);

        if (mpc.exportCode( "acado_mpc_export" ) != SUCCESSFUL_RETURN)
                exit( EXIT_FAILURE );

        mpc.printDimensionsQP( );

        return EXIT_SUCCESS;
}




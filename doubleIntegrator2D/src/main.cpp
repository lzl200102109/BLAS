#include"main.h"

static void populatebyrow (IloModel model, IloNumVarArray x, IloRangeArray c);



int main (int argc, char **argv)
{

  double  dt  = 0.1,
          c_t = 0.5,
          m   = 1.5;

  double A_c[N*N] = { 0, 0, 1, 0,
                      0, 0, 0, 1,
                      0, 0, -c_t/m, 0,
                      0, 0, 0, -c_t/m },

         B_c[N*M] = { 0, 0,
                      0, 0,
                      1, 0,
                      0, 1 },

         I[N*N] = { 1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1 };

  double A[N*N],
         B[N*M];

  copy(A_c, A_c+N*N, A);
  copy(B_c, B_c+N*M, B);

  // compute discretized matrices A and B by Euler approximation
  // A = A_c*dt + I;
  // B = B_c*dt;
  scalarMatMulti(A, dt, N, N);
  matrixSum(I, A, A, N, N);
  scalarMatMulti(B, dt, N, M);

//  printMatrix(A, N, N);
//  cout << endl;
//  printMatrix(B, N, M);
//  cout << endl;


  // solve the optimization problem
  IloEnv   env;
  try {
     IloModel model(env);
     IloNumVarArray var(env);
     IloRangeArray con(env);

     populatebyrow (model, var, con);

     IloCplex cplex(model);
     cplex.exportModel("lpex1.lp");
     cplex.setOut(env.getNullStream());

     // Optimize the problem and obtain solution.
     if ( !cplex.solve() ) {
        env.error() << "Failed to optimize LP" << endl;
        throw(-1);
     }

     IloNumArray vals(env);
     env.out() << "Solution status = " << cplex.getStatus() << endl;
     env.out() << "Solution value  = " << cplex.getObjValue() << endl;
     cplex.getValues(vals, var);
     env.out() << "Values        = " << vals << endl;
     cplex.getSlacks(vals, con);
     env.out() << "Slacks        = " << vals << endl;
     cplex.getDuals(vals, con);
     env.out() << "Duals         = " << vals << endl;
     cplex.getReducedCosts(vals, var);
     env.out() << "Reduced Costs = " << vals << endl;
  }
  catch (IloException& e) {
     cerr << "Concert exception caught: " << e << endl;
  }
  catch (...) {
     cerr << "Unknown exception caught" << endl;
  }

  env.end();

  return 0;
}


static void populatebyrow (IloModel model, IloNumVarArray x, IloRangeArray c)
{
   IloEnv env = model.getEnv();

   x.add(IloNumVar(env, 0.0, 40.0));
   x.add(IloNumVar(env));
   x.add(IloNumVar(env));

   model.add(IloMaximize(env, x[0] + 2 * x[1] + 3 * x[2]));

   c.add( - x[0] +     x[1] + x[2] <= 20);
   c.add(   x[0] - 3 * x[1] + x[2] <= 30);

   x[0].setName("x1");
   x[1].setName("x2");
   x[2].setName("x3");

   c[0].setName("c1");
   c[1].setName("c2");
   model.add(c);

}

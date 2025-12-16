#include "gurobi_c++.h"

vector<bool> solve_min_switch_set_cover_GUROBI(
    int nFlows,
    int nSwitches,
    const vector<vector<int>>& paths
) {
    try {
        // 1. Create environment and model
        GRBEnv env = GRBEnv(true);
        env.set("LogFile", "gurobi.log");
        env.start();

        GRBModel model = GRBModel(env);
        model.set(GRB_StringAttr_ModelName, "switch_set_cover");

        // 2. Decision variables: x[j] ∈ {0,1}
        vector<GRBVar> x(nSwitches);
        for (int j = 0; j < nSwitches; ++j) {
            x[j] = model.addVar(
                0.0, 1.0, 1.0, GRB_BINARY,
                "switch_" + to_string(j)
            );
        }

        // 3. Constraints: each flow must be covered
        for (int i = 0; i < nFlows; ++i) {
            GRBLinExpr expr = 0;
            for (int sw : paths[i]) {
                if (sw < 0 || sw >= nSwitches) {
                    throw runtime_error("Switch index out of range");
                }
                expr += x[sw];
            }
            model.addConstr(expr >= 1, "flow_" + to_string(i));
        }

        // 4. Objective: minimize number of switches
        model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

        // 5. Solver parameters (important!)
        model.set(GRB_DoubleParam_TimeLimit, 120.0);   // seconds
        model.set(GRB_DoubleParam_MIPGap, 0.01);       // 1% gap
        model.set(GRB_IntParam_Presolve, 2);
        model.set(GRB_IntParam_Threads, 0);            // use all cores

        // 6. Optimize
        model.optimize();

        // 7. Check solution
        int status = model.get(GRB_IntAttr_Status);
        if (status != GRB_OPTIMAL && status != GRB_TIME_LIMIT) {
            throw runtime_error("No feasible solution found");
        }

        // 8. Extract solution
        vector<bool> chosen(nSwitches, false);
        for (int j = 0; j < nSwitches; ++j) {
            chosen[j] = (x[j].get(GRB_DoubleAttr_X) > 0.5);
        }

        double obj = model.get(GRB_DoubleAttr_ObjVal);
        cout << "\nBest solution found = " << obj << " switches\n";

        double gap = model.get(GRB_DoubleAttr_MIPGap);
        cout << "Final MIP gap = " << gap * 100 << "%\n";

        return chosen;
    }
    catch (GRBException& e) {
        cerr << "Gurobi error: " << e.getMessage() << endl;
        exit(1);
    }
    catch (...) {
        cerr << "Unknown error during optimization\n";
        exit(1);
    }
}

int main(int argc, char* argv[]) {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    string filename = "flows_10k_100s.csv";
    if (argc >= 2) {
        filename = argv[1];
    }

    int nFlows, nSwitches;
    vector<vector<int>> paths;

    // 1. Load dataset
    load_dataset_csv(filename, nFlows, nSwitches, paths);

    // 2. Solve ILP for minimum switch set cover
    vector<bool> chosen = solve_min_switch_set_cover_GUROBI(nFlows, nSwitches, paths);

    // 3. Print chosen switches
    cout << "\nChosen switches (minimal set cover):" << endl;
    int countChosen = 0;
    for (int j = 0; j < nSwitches; ++j) {
        if (chosen[j]) {
            cout << "  switch " << j << "\n";
            countChosen++;
        }
    }
    cout << "Total chosen switches = " << countChosen << " / " << nSwitches << endl;

    // 4. Validate coverage
    int coveredFlows = count_covered_flows(nFlows, paths, chosen);
    cout << "Flows actually covered = " << coveredFlows
         << " / " << nFlows << endl;

    if (coveredFlows != nFlows) {
        cout << "Warning: not all flows are covered! (may be due to empty paths)\n";
    }

    // 5. Optionally, write selected switches to a file
    ofstream fout("selected_switches.txt");
    for (int j = 0; j < nSwitches; ++j) {
        if (chosen[j]) fout << j << "\n";
    }
    fout.close();
    cout << "Selected switches written to selected_switches.txt\n";

    return 0;
}

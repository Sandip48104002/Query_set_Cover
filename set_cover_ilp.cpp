#include <bits/stdc++.h>
#include <glpk.h>

using namespace std;

/*
 * Load dataset from CSV:
 * Each line: flow_id,switch_id
 * Assumes 0-based IDs for both.
 *
 * Output:
 *   nFlows     - total number of flows
 *   nSwitches  - total number of switches
 *   paths[i]   - vector of switches on path of flow i
 */
void load_dataset_csv(
    const string& filename,
    int& nFlows,
    int& nSwitches,
    vector<vector<int>>& paths
) {
    ifstream fin(filename);
    if (!fin.is_open()) {
        cerr << "Error: cannot open file " << filename << endl;
        exit(1);
    }

    string line;
    // Skip header
    if (!getline(fin, line)) {
        cerr << "Error: empty file " << filename << endl;
        exit(1);
    }

    vector<pair<int,int>> edges;
    int maxFlow = -1, maxSwitch = -1;

    while (getline(fin, line)) {
        if (line.empty()) continue;
        stringstream ss(line);
        string f, s;
        if (!getline(ss, f, ',')) continue;
        if (!getline(ss, s, ',')) continue;
        if (f.empty() || s.empty()) continue;

        int flow = stoi(f);
        int sw   = stoi(s);

        edges.push_back({flow, sw});
        maxFlow   = max(maxFlow, flow);
        maxSwitch = max(maxSwitch, sw);
    }

    fin.close();

    if (maxFlow < 0 || maxSwitch < 0) {
        cerr << "Error: no valid edges parsed from file " << filename << endl;
        exit(1);
    }

    nFlows = maxFlow + 1;
    nSwitches = maxSwitch + 1;

    paths.assign(nFlows, {});
    for (auto &e : edges) {
        int flow = e.first;
        int sw   = e.second;
        paths[flow].push_back(sw);
    }

    // Optional: warn if some flows have empty paths
    for (int i = 0; i < nFlows; ++i) {
        if (paths[i].empty()) {
            cerr << "Warning: flow " << i
                 << " has no switches in its path (constraint will be impossible).\n";
        }
    }

    cout << "Loaded dataset from " << filename << endl;
    cout << "  Flows    : " << nFlows << endl;
    cout << "  Switches : " << nSwitches << endl;
}

/*
 * Solve minimum set of switches (set cover) using ILP with GLPK.
 *
 * nFlows: number of flows
 * nSwitches: number of switches
 * paths[i]: list of switches (0-based indices) on the path of flow i
 *
 * Returns: vector<bool> chooseSwitch[j] = true if switch j is selected.
 */
vector<bool> solve_min_switch_set_cover_ILP(
    int nFlows,
    int nSwitches,
    const vector<vector<int>>& paths
) {
    // Create problem
    glp_prob* lp = glp_create_prob();
    glp_set_prob_name(lp, "switch_set_cover");
    glp_set_obj_dir(lp, GLP_MIN); // minimize sum of x_j

    // Add rows (constraints): one per flow
    glp_add_rows(lp, nFlows);
    for (int i = 0; i < nFlows; ++i) {
        // sum_{j in Path(i)} x_j >= 1
        string row_name = "flow_" + to_string(i);
        glp_set_row_name(lp, i + 1, row_name.c_str());
        glp_set_row_bnds(lp, i + 1, GLP_LO, 1.0, 0.0); // lower bound 1
    }

    // Add columns (variables): one per switch
    glp_add_cols(lp, nSwitches);
    for (int j = 0; j < nSwitches; ++j) {
        string col_name = "switch_" + to_string(j);
        glp_set_col_name(lp, j + 1, col_name.c_str());
        // 0 <= x_j <= 1
        glp_set_col_bnds(lp, j + 1, GLP_DB, 0.0, 1.0);
        // Binary variable
        glp_set_col_kind(lp, j + 1, GLP_BV);
        // Objective coefficient: minimize sum x_j
        glp_set_obj_coef(lp, j + 1, 1.0);
    }

    // Build constraint matrix A (rows = flows, cols = switches)
    // A[i, j] = 1 if switch j is on path of flow i, else 0
    // GLPK uses 1-based indexing for matrix arrays.
    int nz = 0;
    for (int i = 0; i < nFlows; ++i) {
        nz += (int)paths[i].size();
    }

    vector<int> ia(nz + 1); // row indices
    vector<int> ja(nz + 1); // column indices
    vector<double> ar(nz + 1); // values

    int k = 1; // matrix index (start from 1)
    for (int i = 0; i < nFlows; ++i) {
        for (int sw : paths[i]) {
            if (sw < 0 || sw >= nSwitches) {
                cerr << "Error: switch index out of range in paths." << endl;
                glp_delete_prob(lp);
                exit(1);
            }
            ia[k] = i + 1;      // row
            ja[k] = sw + 1;     // column
            ar[k] = 1.0;        // coefficient
            ++k;
        }
    }

    glp_load_matrix(lp, nz, ia.data(), ja.data(), ar.data());

    // Solve as MIP
    glp_iocp parm;
    glp_init_iocp(&parm);
    parm.presolve = GLP_ON;

    int status = glp_intopt(lp, &parm);
    if (status != 0) {
        cerr << "GLPK intopt failed with status " << status << endl;
        glp_delete_prob(lp);
        exit(1);
    }

    int mipStatus = glp_mip_status(lp);
    if (mipStatus != GLP_OPT) {
        cerr << "No optimal MIP solution found, status = " << mipStatus << endl;
        glp_delete_prob(lp);
        exit(1);
    }

    // Extract solution
    vector<bool> chosen(nSwitches, false);
    for (int j = 0; j < nSwitches; ++j) {
        double val = glp_mip_col_val(lp, j + 1);
        chosen[j] = (val > 0.5); // binary
    }

    double opt = glp_mip_obj_val(lp);
    cout << "\nOptimal number of switches (objective) = " << opt << endl;

    glp_delete_prob(lp);
    return chosen;
}

/*
 * Verify how many flows are actually covered by the chosen switches.
 * Full coverage set cover => should be nFlows.
 */
int count_covered_flows(
    int nFlows,
    const vector<vector<int>>& paths,
    const vector<bool>& chosen
) {
    vector<bool> covered(nFlows, false);

    for (int f = 0; f < nFlows; ++f) {
        for (int sw : paths[f]) {
            if (sw >= 0 && sw < (int)chosen.size() && chosen[sw]) {
                covered[f] = true;
                break;
            }
        }
    }

    int cnt = 0;
    for (bool c : covered) if (c) cnt++;
    return cnt;
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
    vector<bool> chosen = solve_min_switch_set_cover_ILP(nFlows, nSwitches, paths);

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

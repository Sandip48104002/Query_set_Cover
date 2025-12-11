#include <bits/stdc++.h>
using namespace std;

/*
 * Load dataset from CSV:
 * Each line: flow_id,switch_id
 * Assumes 0-based IDs.
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
    getline(fin, line);

    vector<pair<int,int>> edges;
    int maxFlow = -1, maxSwitch = -1;

    while (getline(fin, line)) {
        if (line.empty()) continue;
        stringstream ss(line);
        string f, s;
        getline(ss, f, ',');
        getline(ss, s, ',');

        int flow = stoi(f);
        int sw   = stoi(s);

        edges.push_back({flow, sw});
        maxFlow   = max(maxFlow, flow);
        maxSwitch = max(maxSwitch, sw);
    }
    fin.close();

    nFlows = maxFlow + 1;
    nSwitches = maxSwitch + 1;

    paths.assign(nFlows, {});
    for (auto &e : edges)
        paths[e.first].push_back(e.second);

    cout << "Loaded dataset:" << endl;
    cout << "  Flows    : " << nFlows << endl;
    cout << "  Switches : " << nSwitches << endl;
}

/*
 * Greedy Set Cover for minimum switches.
 */
vector<bool> greedy_set_cover(
    int nFlows,
    int nSwitches,
    const vector<vector<int>>& paths
) {
    // Build reverse mapping: switch -> flows
    vector<vector<int>> switchFlows(nSwitches);
    for (int f = 0; f < nFlows; ++f) {
        for (int sw : paths[f]) {
            switchFlows[sw].push_back(f);
        }
    }

    vector<bool> covered(nFlows, false);
    vector<bool> chosen(nSwitches, false);

    int coveredCount = 0;
    int step = 0;

    while (coveredCount < nFlows) {
        int bestSwitch = -1;
        int bestGain   = -1;

        // Find switch with maximum marginal gain
        for (int s = 0; s < nSwitches; ++s) {
            if (chosen[s]) continue;

            int gain = 0;
            for (int f : switchFlows[s]) {
                if (!covered[f]) gain++;
            }

            if (gain > bestGain) {
                bestGain = gain;
                bestSwitch = s;
            }
        }

        // Safety check
        if (bestSwitch == -1 || bestGain == 0) {
            cerr << "Error: remaining flows cannot be covered." << endl;
            break;
        }

        // Select best switch
        chosen[bestSwitch] = true;
        step++;

        // Mark its flows as covered
        for (int f : switchFlows[bestSwitch]) {
            if (!covered[f]) {
                covered[f] = true;
                coveredCount++;
            }
        }

        cout << "Step " << step
             << ": chose switch " << bestSwitch
             << " (gain = " << bestGain
             << ", total covered = " << coveredCount << "/" << nFlows << ")\n";
    }

    return chosen;
}

int main(int argc, char* argv[]) {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    string filename = "flows_10k_100s.csv";
    if (argc >= 2)
        filename = argv[1];

    int nFlows, nSwitches;
    vector<vector<int>> paths;

    // Load dataset
    load_dataset_csv(filename, nFlows, nSwitches, paths);

    auto start = chrono::high_resolution_clock::now();

    // Run greedy
    vector<bool> chosen = greedy_set_cover(nFlows, nSwitches, paths);

    auto end = chrono::high_resolution_clock::now();
    double runtime =
        chrono::duration_cast<chrono::duration<double>>(end - start).count();

    // Print result
    int cnt = 0;
    cout << "\nChosen switches (greedy solution):\n";
    for (int j = 0; j < nSwitches; ++j) {
        if (chosen[j]) {
            cout << "  switch " << j << "\n";
            cnt++;
        }
    }

    cout << "Total chosen switches = " << cnt << endl;
    cout << "Greedy runtime = " << runtime << " seconds\n";

    // Save selected switches
    ofstream fout("selected_switches_greedy.txt");
    for (int j = 0; j < nSwitches; ++j) {
        if (chosen[j]) fout << j << "\n";
    }
    fout.close();

    cout << "Selected switches written to selected_switches_greedy.txt\n";

    return 0;
}

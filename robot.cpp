#include<iostream>
#include<cstring>
using namespace std;

#define N 3
#define INF 10000000

void AddToTree(int, int);
void UpdateLabels();

int cost[N][N];  // cost matrix
int num_robots = N;  // number of rows
int num_jobs   = N;  // number of cols
int max_match  = 0;  // if max_match is N, we are done
int label_x[N], label_y[N];  // vertex labels
int xy[N];  // xy[x] - vertex in Y matched with x
int yx[N];  // yx[x] - vertex in X matched with y
bool S[N], T[N];  // Helps find augmenting path
int slack[N];  // helps update label
int slackx[N]; // slack[y] -> x such that l(x) + l(y) - w(x,y) = slack[y]
int prev[N];  // for memorizing alternating paths

// Step 0 Label
// empty matching
void InitLabels() {
  memset(label_x, 0, sizeof(label_x));
  memset(label_y, 0, sizeof(label_y));
  for (int x=0; x<num_robots; x++) {
    for (int y=0; y<num_jobs; y++) {
      label_x[x] = max(label_x[x], cost[x][y]);
    }
  }
}

void Augment() {
  if (max_match == num_robots) return; // check if matching is perfect
  int x, y, root;
  int q[N], wr = 0, rd = 0;


  memset(S, false, sizeof(S));
  memset(T, false, sizeof(T)); // init S, T
  memset(prev, -1, sizeof(prev)); // init prev for the alternating tree
  // -1 means no prev. -2 means current is root

  // find ONE unmatched x and make it root
  for (x=0; x<num_robots; x++) {
    // finding the exposed x in X and make it the root of the alternating tree
    if (xy[x] == -1) {
      // if x is unmatched
      q[wr++] = root = x;  // find roots of the tree
      prev[x] = -2;  // root has no prev
      S[x] = true;   // put x in S
      break;
    }
  }

  // initialize slack to save computations
  for (y=0; y<num_robots; y++) {
    // Now T is empty, so we find all the slacks of y in Y
    slack[y] = label_x[root] + label_y[y] - cost[root][y]; // help find its min value
    // For all y, slackx[y] can be computed with root
    slackx[y] = root;
  }

  while (true) {
    // look for augmenting path
    while (rd < wr) {
      // iterating over all possible roots
      x = q[rd++];  // one root.   q is for building tree with bfs search
      for (y=0; y<num_robots; y++) {
	if (cost[x][y] == label_x[x] + label_y[y] && !T[y]) { // for all edges
	  if (yx[y] == -1) break;  // an exposed vertex in Y found. augmenting path.
	  T[y] = true;  // add y to T
	  q[wr++] = yx[y];  //
	  AddToTree(yx[y], x);  // put yx[y] in S because y is matched to yx[y]
	}
      }
      if (y<num_robots) break;
    }
    if (y<num_robots) break;

    // Execute the following code only if we fail to find an augmenting path

    UpdateLabels();
    wr = rd = 0;

    // We have updated the labels, but we still need to add edges to the tree
    // y is the newly added element to improve the labeling
    for (y=0; y<num_robots; y++) {
      if (!T[y] && slack[y] == 0) {  // augmenting path
	if (yx[y] == -1) {  // not matched
	  x = slackx[y];
	  break;
	}
	else {
	  T[y] = true;  // y is matched
	  if (!S[yx[y]]) {  // if the one matched to y, r, is not in S
	    q[wr++] = yx[y];  // put r in q
	    // Add r and the v which minimizes l(y)
	    AddToTree(yx[y], slackx[y]);
	  }
	}
      }
    }
    if (y < num_robots) break;
  }

  // augmenting path found, reverse all edges along the path
  if (y<num_robots) {
    max_match++;  // increment
    // y is the unconnected vertex, the target of the augmenting path
    // x is slackx[y]
    // we traverse backward until cx == -2 which is the root
    for (int cx=x, cy=y, ty; cx != -2; cx = prev[cx], cy =ty) {
      ty = xy[cx];  // ty is connected with cx (current x)
      yx[cy] = cx;
      xy[cx] = cy;  // connect cx and cy
    }
    Augment();
  }
}

void UpdateLabels() {
  int x, y, delta = INF;
  for (y=0; y<num_robots; y++) {
    if (!T[y])  // y not in T
      delta = min(delta, slack[y]);  // find the minimum
  }
  for (x=0; x<num_robots; x++)
    if (S[x]) label_x[x] -= delta;
  for (y=0; y<num_robots; y++)
    if (T[y]) label_y[y] += delta;  // update x in S, y in T
  for (y=0; y<num_robots; y++)
    if (!T[y])
      slack[y] -= delta;  // This is because l(x) is delta less, l(y) is constant
}


void AddToTree(int x, int prevx) {
  // x     - current vertex
  // prevx - vertext from X before x in the alternating path
  S[x] = true;  // add x to S
  prev[x] = prevx;  // put it in the alternating path
  for (int y=0; y<num_robots; y++) {
    if (label_x[x] + label_y[y] - cost[x][y] < slack[y]) {
      // update slack because we added a new element to S
      // because slack[y] will always be the minimum from y to ANY element x in X
      slack[y]  = label_x[x] + label_y[y] - cost[x][y];
      slackx[y] = x;
    }
  }
}


int Hungarian() {
  int result = 0;
  max_match  = 0;
  memset(xy, -1, sizeof(xy));
  memset(yx, -1, sizeof(yx)); // initially, no elements are connected
  InitLabels();
  Augment();
  for (int x=0; x<num_robots; x++) {
    cout << x << " is connected to " << xy[x] << endl;
    result += cost[x][xy[x]]; // add up the corresponding costs
  }
  return result;
}


void ReadMatrixFromInput() {
  for (int row=0; row<N; row++) {
    for (int col=0; col<N; col++) {
      cin >> cost[row][col];
    }
  }
}

int FindLargestInMatrix(int A[N][N]) {
  int largest = A[0][0];
  for (int row=0; row<N; row++) {
    for (int col=0; col<N; col++) {
      if (A[row][col] >= largest) {
	largest = A[row][col];
      }
    }
  }
  return largest;
}

void ProcessMatrix(int A[N][N], int largest) {
  for (int row=0; row<N; row++) {
    for (int col=0; col<N; col++) {
      A[row][col] = largest - A[row][col];
    }
  }
}

int main() {
  int largest;
  ReadMatrixFromInput();
  largest = FindLargestInMatrix(cost);
  ProcessMatrix(cost, largest);
  cout << "The minimum cost is: " << N*largest - Hungarian() << endl;
  return 0;
}

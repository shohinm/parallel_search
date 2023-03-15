#include <curses.h>
#include <iostream>
#include <sys/select.h>
#include <termios.h>

#define _countof(array) (sizeof(array) / sizeof(array[0]))

using namespace std;

const int BITS = 3; // increase this if it doesn't find a solution
const int MASK = (1 << BITS) - 1;

struct OBJECT { int id; char* name; } objects[] = {
  0, "pedestal", // default at id 0
  1, "obstacles",
  2, "base",
  3, "link1",
  4, "link2",
  5, "link3",
  6, "link4",
  7, "link5",
  8, "link6",
  9, "shield",
};
struct NO_COLLISION { int o1, o2; } no_collisions[] = {
  0, 1,
  0, 2,
  0, 3,
  0, 4,
  1, 2,
  1, 3,
  1, 4,
  2, 3,
  2, 4,
  2, 5,
  3, 4,
  4, 5,
  4, 6,
  4, 7,
  4, 8,
  4, 9,
  5, 6,
  5, 7,
  5, 8,
  5, 9,
  6, 7,
  6, 8,
  7, 8,
  7, 9,
  8, 9,
};
const int NO = _countof(objects);
const int NNC = _countof(no_collisions);
bool forbidden[NO][NO];
//unsigned int combi;
unsigned long combi;
//unsigned long long combi;

//int _kbhit() {
//  static const int STDIN = 0;
//  static bool initialized = false;
//
//  if (! initialized) {
//    // Use termios to turn off line buffering
//    termios term;
//    tcgetattr(STDIN, &term);
//    term.c_lflag &= ~ICANON;
//    tcsetattr(STDIN, TCSANOW, &term);
//    setbuf(stdin, NULL);
//    initialized = true;
//  }
//
//  int bytesWaiting;
//  ioctl(STDIN, FIONREAD, &bytesWaiting);
//  return bytesWaiting;
//}

void verify_data() {
  for (int i = 0; i < NO; i++) {
    if (objects[i].id != i) throw "?ID SEQUENCE ERROR in objects[]";
  }
  for (int i = 0; i < NNC; i++) {
    if ((unsigned)no_collisions[i].o1 >= NO || (unsigned)no_collisions[i].o2 >= NO) throw "?OUT OF RANGE ERROR in no_collisions[]";
  }
  if (sizeof(combi) * 8 < NO * 2 * BITS) throw "sizeof(combi) too small, use __int64";
}

void init_forbidden() {
  cout << "No collisions between:" << endl;
  for (int i = 0; i < NNC; i++) {
    NO_COLLISION& nc = no_collisions[i];
    forbidden[nc.o1][nc.o2] = forbidden[nc.o2][nc.o1] = true;
    cout << "- " << objects[nc.o1].name << " and " << objects[nc.o2].name << endl;
  }
}

bool search_for_solution() {
  for (combi = 1 | (1 << BITS); combi < INT64_C(1) << NO * 2 * BITS; combi += 1 << 2 * BITS) {
    for (int o1 = 0; o1 < NO; o1++) {
      int contype1 = int(combi >> o1 * 2 * BITS) & MASK;
      int conaffinity1 = int(combi >> o1 * 2 * BITS >> BITS) & MASK;
      for (int o2 = o1; o2 < NO; o2++) {
        int contype2 = int(combi >> o2 * 2 * BITS) & MASK;
        int conaffinity2 = int(combi >> o2 * 2 * BITS >> BITS) & MASK;
        bool collision = (contype1 & conaffinity2 || contype2 & conaffinity1);
        if (collision == forbidden[o1][o2]) goto next_combination;
      }
    }
    return true;
  next_combination:
    ;
  }
  return false;
}

void output_solution() {
  cout << "solution found" << endl;
  cout << "contype,conaffinity:" << endl;
  for (int o1 = 0; o1 < NO; o1++) {
    int contype = int(combi >> o1 * 2 * BITS) & MASK;
    int conaffinity = int(combi >> o1 * 2 * BITS >> BITS) & MASK;
    cout << "  " << contype << "," << conaffinity << " for " << objects[o1].name << endl;
  }
}

int main(int argc, char* argv[]) {
  try {
    verify_data();
    init_forbidden();
    if (search_for_solution()) output_solution();
    else cout << "No solution found, increase BITS" << endl;
  } catch (const char* msg) {
    cerr << endl << msg;
  }
//  cout << endl << "READY." << endl; do getch(); while (_kbhit());
  return 0;
}

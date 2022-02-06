#include <ilcplex/ilocplex.h>
#include <iostream>
#include <queue>
#include <time.h>
#include <tuple>
#include <fstream>
#include <sstream>
#include <string> 
#include <stdlib.h>     
#include <vector>
#include <chrono>
#include <random>
#include <math.h>       
#include <stack> 
#include <list>
#include <cstdlib> 
#include <time.h>
#include <ctime> 
#include <cmath>

ILOSTLBEGIN

using namespace std;
using namespace std::chrono;


int n = 135; // all elevators + requests + destinations (dimension of cost matrix)
int m = 20; // number of elevators
int l[20] = { 1,2,7,10,12,14,15,17,135,19,21,23,24,29,33,35,37,41,44,48 }; // elevator locations


int Elevator_Requests[9] = { 3,8,13,20,22,36,38,42,49 };
int Elevator_Requests_Sources[9] = { 12,17,22,29,30,43,44,48,8 };

int U_Max = 134;
bool flag_repetition = false;
bool check_l[20] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
int r[115] = { 4,5,6,9,11,16,18,25,26,27,28,30,31,32,34,39,40,43,45,46,47,3,8,13,20,22,36,38,42,49,50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125,126,127,128,129,130,131,132,133,134 };// requests+destinations
int r2[21] = { 4,5,6,9,11,16,18,25,26,27,28,30,31,32,34,39,40,43,45,46,47 }; // just requests
int d[85] = { 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125,126,127,128,129,130,131,132,133,134 }; // destinations

																																																																																															  /* 30 users for 300 floor (hours) */
																																																																																															  //int PoissonArrival[30] = { 0, 3, 3, 14, 16, 19, 24, 35, 80, 86, 136, 185, 188, 189, 190, 209, 218, 243, 279, 289, 297, 298, 298, 314, 326, 340, 348, 360, 370, 392 };
int AllRequests[30] = { 4,5,22,6,9,11,20,16,49,18,25,26,3,27,28,30,31,32,36,34,39,13,40,42,43,45,8,46,38,47 }; /* including elevator requests */
int AllRequests_Sources[30] = { 13,14,30,15,18,21,29,25,8,28,33,34,12,35,37,39,3,40,43,42,45,22,47,48,49,50,17,5,44,7 }; /* including elevator requests */
bool AllRequests_Types[30] = { 1,1,0,1,1,1,0,1,0,1,1,1,0,1,1,1,1,1,0,1,1,0,1,0,1,1,0,1,0,1 }; /* 1 indicates only request and 0 indicates elevator requests  */

int PoissonArrival[30] = { 1,1,1,1,1,1,2,2,3,4,5,6,6,6,6,7,7,8,9,9,10,10,10,10,11,11,11,12,12,13 };
bool flag_round = false;

int p = 11; // number of stops

float n1 = 135;
float p1 = 11;

/* source floor */
int E[20] = { 10,12,17,19,22,23,24,27,27,29,2,30,31,38,41,43,44,48,4,8 };


int R[21] = { 13,14,15,18,21,25,28,33,34,35,37,39,3,40,42,45,47,49,50,5,7 };
int D[85] = { 3,3,3,3,3,5,7,8,8,8,8,8,12,12,13,13,13,13,14,14,15,15,15,17,17,17,17,17,18,18,18,18,21,21,22,22,22,22,25,25,25,25,28,28,28,29,29,29,29,30,30,33,34,34,34,34,34,35,35,35,35,37,37,39,39,40,40,40,42,43,43,43,43,44,44,44,44,45,45,47,48,49,49,50,50 };
int DF[85] = { 48,12,7,31,22,25,14,42,10,15,25,4,22,18,40,50,47,18,19,22,27,46,42,6,3,34,4,49,7,20,37,4,49,20,11,34,15,37,9,15,45,14,45,12,21,18,27,39,25,45,9,17,37,11,27,31,40,48,21,15,7,15,9,31,50,46,43,14,42,20,41,11,19,25,43,46,19,10,31,13,17,5,48,11,18 };

/* Result of MST used for DFS */
int MST_sources[135];
int MST_destinations[135];
int MST_count = 0;

int maxint = 600;

int Num_SelectedEdges = 0;

int C[134][134] = {
	{ maxint, maxint, 3, 4, 5, maxint, 8, maxint, 12, maxint, maxint, maxint, 16, maxint, 19, maxint, maxint, maxint, maxint, 25, 26, 27, 29, maxint, 31, 7, 33, maxint, 35, maxint, maxint, 38, 40, maxint, 42, maxint, 44, 5, 3, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 1, 2, 3, maxint, 6, maxint, 9, maxint, maxint, maxint, 14, maxint, 17, maxint, maxint, maxint, maxint, 23, 24, 25, 27, maxint, 29, 9, 30, maxint, 33, maxint, maxint, 36, 38, maxint, 40, maxint, 41, 7, 5, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 1, 2, 3, maxint, 6, maxint, 9, maxint, maxint, maxint, 14, maxint, 17, maxint, maxint, maxint, maxint, 23, 24, 25, 27, maxint, 29, 9, 30, maxint, 33, maxint, maxint, 36, 38, maxint, 40, maxint, 41, 7, 5, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, maxint, 1, 2, 0, 5, 0, 8, 0, 0, 0, 13, 0, 16, 0, 0, 0, 0, 22, 23, 24, 26, 0, 28, 11, 29, 0, 31, 0, 0, 35, 37, 0, 39, 0, 40, 8, 6, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 29, 40, 37, 5, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 1, maxint, 1, 0, 4, 0, 7, 0, 0, 0, 12, 0, 15, 0, 0, 0, 0, 20, 22, 23, 25, 0, 27, 12, 28, 0, 30, 0, 0, 34, 36, 0, 38, 0, 39, 9, 7, 0, maxint, maxint, 7, maxint, maxint, 12, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, 28, maxint, maxint, maxint, 5, 8, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 7, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 28, maxint, maxint, maxint, 7, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 2, 1, maxint, 0, 3, 0, 6, 0, 0, 0, 11, 0, 14, 0, 0, 0, 0, 19, 20, 22, 24, 0, 26, 13, 27, 0, 29, 0, 0, 33, 35, 0, 37, 0, 38, 11, 8, 0, maxint, maxint, maxint, maxint, 7, 11, maxint, maxint, maxint, maxint, 11, maxint, 7, maxint, maxint, maxint, maxint, maxint, maxint, 7, 13, 34, 29, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 24, maxint, maxint, maxint, maxint, maxint, maxint, 24, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, maxint, maxint, maxint, 24, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 4, 3, 2, maxint, 1, maxint, 4, maxint, maxint, maxint, 8, maxint, 12, maxint, maxint, maxint, maxint, 17, 18, 19, 22, maxint, 24, 15, 25, maxint, 27, maxint, maxint, 30, 33, maxint, 35, maxint, 36, 13, 11, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, 15, 18, 14, 35, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 4, 3, 2, maxint, 1, maxint, 4, maxint, maxint, maxint, 8, maxint, 12, maxint, maxint, maxint, maxint, 17, 18, 19, 22, maxint, 24, 15, 25, maxint, 27, maxint, maxint, 30, 33, maxint, 35, maxint, 36, 13, 11, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, 15, 18, 14, 35, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 5, 4, 3, 0, maxint, 0, 3, 0, 0, 0, 7, 0, 11, 0, 0, 0, 0, 16, 17, 18, 20, 0, 23, 16, 24, 0, 26, 0, 0, 29, 31, 0, 34, 0, 35, 14, 12, 0, maxint, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 35, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, 2, 20, 15, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 35, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 5, maxint, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 6, 5, 4, maxint, 1, maxint, 2, maxint, maxint, maxint, 6, maxint, 9, maxint, maxint, maxint, maxint, 15, 16, 17, 19, maxint, 22, 17, 23, maxint, 25, maxint, maxint, 28, 30, maxint, 33, maxint, 34, 15, 13, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 8, 7, 6, 0, 3, 0, maxint, 0, 0, 0, 4, 0, 7, 0, 0, 0, 0, 13, 14, 15, 17, 0, 19, 19, 20, 0, 23, 0, 0, 26, 28, 0, 30, 0, 31, 17, 15, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 30, 1, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 9, 8, 7, maxint, 4, maxint, 1, maxint, maxint, maxint, 3, maxint, 6, maxint, maxint, maxint, maxint, 12, 13, 14, 16, maxint, 18, 20, 19, maxint, 22, maxint, maxint, 25, 27, maxint, 29, maxint, 30, 18, 16, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, 13, 7, 16, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 9, 8, 7, maxint, 4, maxint, 1, maxint, maxint, maxint, 3, maxint, 6, maxint, maxint, maxint, maxint, 12, 13, 14, 16, maxint, 18, 20, 19, maxint, 22, maxint, maxint, 25, 27, maxint, 29, maxint, 30, 18, 16, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, 13, 7, 16, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 10, 9, 8, maxint, 5, maxint, 2, maxint, maxint, maxint, 2, maxint, 5, maxint, maxint, maxint, maxint, 11, 12, 13, 15, maxint, 17, 22, 18, maxint, 20, maxint, maxint, 24, 26, maxint, 28, maxint, 29, 19, 17, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 11, 10, 9, maxint, 6, maxint, 3, maxint, maxint, maxint, 1, maxint, 4, maxint, maxint, maxint, maxint, 9, 11, 12, 14, maxint, 16, 23, 17, maxint, 19, maxint, maxint, 23, 25, maxint, 27, maxint, 28, 20, 18, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 12, 11, 10, 0, 7, 0, 4, 0, 0, 0, maxint, 0, 3, 0, 0, 0, 0, 8, 9, 11, 13, 0, 15, 24, 16, 0, 18, 0, 0, 22, 24, 0, 26, 0, 27, 22, 19, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 17, 11, 22, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 22, maxint, maxint, maxint },
	{ maxint, maxint, 14, 13, 12, maxint, 9, maxint, 6, maxint, maxint, maxint, 2, maxint, 1, maxint, maxint, maxint, maxint, 6, 7, 8, 11, maxint, 13, 26, 14, maxint, 16, maxint, maxint, 19, 22, maxint, 24, maxint, 25, 24, 22, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 15, 14, 13, 0, 10, 0, 7, 0, 0, 0, 3, 0, maxint, 0, 0, 0, 0, 5, 6, 7, 9, 0, 12, 27, 13, 0, 15, 0, 0, 18, 20, 0, 23, 0, 24, 25, 23, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 18, 17, 7, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 16, 15, 14, maxint, 11, maxint, 8, maxint, maxint, maxint, 4, maxint, 1, maxint, maxint, maxint, maxint, 4, 5, 6, 8, maxint, 11, 28, 12, maxint, 14, maxint, maxint, 17, 19, maxint, 22, maxint, 23, 26, 24, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, 2, 11, 4, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 16, 15, 14, maxint, 11, maxint, 8, maxint, maxint, maxint, 4, maxint, 1, maxint, maxint, maxint, maxint, 4, 5, 6, 8, maxint, 11, 28, 12, maxint, 14, maxint, maxint, 17, 19, maxint, 22, maxint, 23, 26, 24, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, 2, 11, 4, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 11, 12, 13, maxint, 16, maxint, 19, maxint, maxint, maxint, 23, maxint, 26, maxint, maxint, maxint, maxint, 34, 35, 36, 38, maxint, 40, 1, 41, maxint, 44, maxint, maxint, 47, 49, maxint, 51, maxint, 52, 3, 5, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 11, 12, 13, maxint, 16, maxint, 19, maxint, maxint, maxint, 23, maxint, 26, maxint, maxint, maxint, maxint, 34, 35, 36, 38, maxint, 40, 1, 41, maxint, 44, maxint, maxint, 47, 49, maxint, 51, maxint, 52, 3, 5, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 17, 16, 15, maxint, 12, maxint, 9, maxint, maxint, maxint, 5, maxint, 2, maxint, maxint, maxint, maxint, 3, 4, 5, 7, maxint, 9, 29, 11, maxint, 13, maxint, maxint, 16, 18, maxint, 20, maxint, 22, 27, 25, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 16, 23, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 18, 17, 16, maxint, 13, maxint, 10, maxint, maxint, maxint, 6, maxint, 3, maxint, maxint, maxint, maxint, 2, 3, 4, 6, maxint, 8, 30, 9, maxint, 12, maxint, maxint, 15, 17, maxint, 19, maxint, 20, 28, 26, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 20, 19, 18, 0, 15, 0, 12, 0, 0, 0, 8, 0, 5, 0, 0, 0, 0, maxint, 1, 2, 4, 0, 6, 33, 7, 0, 9, 0, 0, 13, 15, 0, 17, 0, 18, 30, 28, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 17, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 21, 20, 19, 0, 16, 0, 13, 0, 0, 0, 9, 0, 6, 0, 0, 0, 0, 1, maxint, 1, 3, 0, 5, 34, 6, 0, 8, 0, 0, 12, 14, 0, 16, 0, 17, 31, 29, 0, maxint, maxint, maxint, maxint, 13, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 13, maxint, maxint, maxint, maxint, maxint, maxint, 13, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 18, 3, 25, 7, 3, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 18, maxint, maxint, maxint, maxint },
	{ 0, 0, 22, 21, 20, 0, 17, 0, 14, 0, 0, 0, 10, 0, 7, 0, 0, 0, 0, 2, 1, maxint, 2, 0, 4, 35, 5, 0, 7, 0, 0, 11, 13, 0, 15, 0, 16, 33, 30, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 14, 15, 22, 30, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 24, 23, 22, 0, 19, 0, 16, 0, 0, 0, 12, 0, 9, 0, 0, 0, 0, 4, 3, 2, maxint, 0, 2, 37, 3, 0, 5, 0, 0, 8, 11, 0, 13, 0, 14, 35, 33, 0, maxint, maxint, maxint, maxint, 16, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 16, 20, maxint, maxint, maxint, 20, maxint, 16, maxint, maxint, maxint, maxint, maxint, 3, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 3, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 20, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 24, 30, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 20 },
	{ maxint, maxint, 25, 24, 23, maxint, 20, maxint, 17, maxint, maxint, maxint, 13, maxint, 10, maxint, maxint, maxint, maxint, 5, 4, 3, 1, maxint, 1, 38, 2, maxint, 4, maxint, maxint, 7, 9, maxint, 12, maxint, 13, 36, 34, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 26, 25, 24, 0, 21, 0, 18, 0, 0, 0, 14, 0, 11, 0, 0, 0, 0, 6, 5, 4, 2, 0, maxint, 39, 1, 0, 3, 0, 0, 6, 8, 0, 11, 0, 12, 37, 35, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 8, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 10, 11, 12, 0, 15, 0, 18, 0, 0, 0, 22, 0, 25, 0, 0, 0, 0, 30, 31, 32, 34, 0, 36, maxint, 40, 0, 42, 0, 0, 46, 48, 0, 50, 0, 51, 2, 4, 0, 49, 9, 4, 30, 20, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 15, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 15, maxint, maxint, maxint, maxint },
	{ 0, 0, 27, 26, 25, 0, 22, 0, 19, 0, 0, 0, 15, 0, 12, 0, 0, 0, 0, 7, 6, 5, 3, 0, 1, 37, maxint, 0, 2, 0, 0, 5, 7, 0, 9, 0, 11, 38, 36, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 6, 3, 28, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 29, maxint, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 28, 27, 26, maxint, 23, maxint, 20, maxint, maxint, maxint, 16, maxint, 13, maxint, maxint, maxint, maxint, 8, 7, 6, 4, maxint, 2, 38, 1, maxint, 1, maxint, maxint, 4, 6, maxint, 8, maxint, 9, 39, 37, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 29, 28, 27, 0, 24, 0, 21, 0, 0, 0, 17, 0, 14, 0, 0, 0, 0, 9, 8, 7, 5, 0, 3, 39, 2, 0, maxint, 0, 0, 3, 5, 0, 7, 0, 8, 40, 38, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 29, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 29, maxint, maxint, 29, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 29, maxint, 29, maxint, maxint, maxint, maxint, maxint, maxint, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 30, 29, 28, maxint, 25, maxint, 22, maxint, maxint, maxint, 18, maxint, 15, maxint, maxint, maxint, maxint, 10, 9, 8, 6, maxint, 4, 40, 3, maxint, 1, maxint, maxint, 2, 4, maxint, 6, maxint, 7, 41, 39, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 25, 2, 35, 26, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 30, 29, 28, maxint, 25, maxint, 22, maxint, maxint, maxint, 18, maxint, 15, maxint, maxint, maxint, maxint, 10, 9, 8, 6, maxint, 4, 40, 3, maxint, 1, maxint, maxint, 2, 4, maxint, 6, maxint, 7, 41, 39, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 25, 2, 35, 26, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 31, 30, 29, maxint, 26, maxint, 23, maxint, maxint, maxint, 19, maxint, 16, maxint, maxint, maxint, maxint, 11, 10, 9, 7, maxint, 5, 41, 4, maxint, 2, maxint, maxint, 1, 3, maxint, 5, maxint, 6, 42, 40, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 20, 1, 2, 27, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 31, 30, 29, maxint, 26, maxint, 23, maxint, maxint, maxint, 19, maxint, 16, maxint, maxint, maxint, maxint, 11, 10, 9, 7, maxint, 5, 41, 4, maxint, 2, maxint, maxint, 1, 3, maxint, 5, maxint, 6, 42, 40, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 20, 1, 2, 27, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 32, 31, 30, 0, 27, 0, 24, 0, 0, 0, 20, 0, 17, 0, 0, 0, 0, 12, 11, 10, 8, 0, 6, 42, 5, 0, 3, 0, 0, maxint, 2, 0, 4, 0, 5, 44, 41, 0, maxint, maxint, maxint, maxint, maxint, 22, maxint, maxint, maxint, maxint, 22, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 22, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 22, maxint, maxint, maxint, 38, 15, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 34, 33, 32, 0, 29, 0, 26, 0, 0, 0, 22, 0, 19, 0, 0, 0, 0, 14, 13, 12, 10, 0, 8, 44, 7, 0, 5, 0, 0, 2, maxint, 0, 2, 0, 3, 46, 44, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 37, maxint, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 35, 34, 33, maxint, 30, maxint, 27, maxint, maxint, maxint, 23, maxint, 20, maxint, maxint, maxint, maxint, 15, 14, 13, 11, maxint, 9, 45, 8, maxint, 6, maxint, maxint, 3, 1, maxint, 1, maxint, 2, 47, 45, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 34, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 35, 34, 33, maxint, 30, maxint, 27, maxint, maxint, maxint, 23, maxint, 20, maxint, maxint, maxint, maxint, 15, 14, 13, 11, maxint, 9, 45, 8, maxint, 6, maxint, maxint, 3, 1, maxint, 1, maxint, 2, 47, 45, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 34, maxint, maxint, maxint, maxint },
	{ 0, 0, 36, 35, 34, 0, 31, 0, 28, 0, 0, 0, 24, 0, 21, 0, 0, 0, 0, 16, 15, 14, 12, 0, 10, 46, 9, 0, 7, 0, 0, 4, 2, 0, maxint, 0, 1, 48, 46, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 30, maxint, maxint, maxint, maxint, maxint, maxint, 35, maxint, maxint, maxint, maxint, maxint, maxint, 30, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 35, 48, 1, maxint, maxint },
	{ maxint, maxint, 9, 10, 11, maxint, 14, maxint, 17, maxint, maxint, maxint, 21, maxint, 24, maxint, maxint, maxint, maxint, 29, 30, 31, 33, maxint, 35, 1, 36, maxint, 38, maxint, maxint, 41, 43, maxint, 45, maxint, 50, 1, 3, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 37, 36, 35, 0, 32, 0, 29, 0, 0, 0, 25, 0, 22, 0, 0, 0, 0, 17, 16, 15, 13, 0, 11, 47, 10, 0, 8, 0, 0, 5, 3, 0, 1, 0, maxint, 49, 47, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 40, maxint, maxint, maxint, 42, 35 },
	{ 0, 0, 8, 9, 10, 0, 13, 0, 16, 0, 0, 0, 20, 0, 23, 0, 0, 0, 0, 28, 29, 30, 32, 0, 34, 2, 35, 0, 37, 0, 0, 40, 42, 0, 44, 0, 45, maxint, 2, 0, maxint, maxint, maxint, maxint, maxint, 22, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 48, maxint, maxint, maxint, maxint, 48, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 6, 7, 8, 0, 11, 0, 14, 0, 0, 0, 18, 0, 21, 0, 0, 0, 0, 26, 27, 28, 30, 0, 32, 4, 33, 0, 35, 0, 0, 38, 40, 0, 42, 0, 43, 2, maxint, 0, maxint, maxint, maxint, maxint, maxint, maxint, 7, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, 4, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12 },
	{ maxint, maxint, 5, 6, 7, maxint, 10, maxint, 13, maxint, maxint, maxint, 17, maxint, 20, maxint, maxint, maxint, maxint, 25, 26, 27, 29, maxint, 31, 5, 32, maxint, 34, maxint, maxint, 37, 39, maxint, 41, maxint, 42, 3, 1, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 37, 2, 7, 18, 4, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ maxint, maxint, 5, 6, 7, maxint, 10, maxint, 13, maxint, maxint, maxint, 17, maxint, 20, maxint, maxint, maxint, maxint, 25, 26, 27, 29, maxint, 31, 5, 32, maxint, 34, maxint, maxint, 37, 39, maxint, 41, maxint, 42, 3, 1, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 37, 2, 7, 18, 4, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 35, 34, 33, 0, 30, 0, 27, 0, 0, 0, 23, 0, 20, 0, 0, 0, 0, 15, 14, 13, 11, 0, 9, 45, 8, 0, 6, 0, 0, 3, 1, 0, 1, 0, 2, 43, 41, 0, maxint, 39, 45, 18, 28, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 49, maxint, maxint, 1, maxint, maxint, maxint, maxint, 1, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 34, maxint, maxint, maxint, maxint },
	{ 0, 0, 1, 2, 3, 0, 6, 0, 9, 0, 0, 0, 13, 0, 16, 0, 0, 0, 0, 21, 22, 23, 25, 0, 27, 9, 28, 0, 30, 0, 0, 33, 35, 0, 37, 0, 38, 7, 5, 0, 36, maxint, 5, 20, 11, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 9, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 6, 7, 8, 0, 11, 0, 14, 0, 0, 0, 18, 0, 21, 0, 0, 0, 0, 26, 27, 28, 30, 0, 32, 4, 33, 0, 35, 0, 0, 38, 40, 0, 42, 0, 43, 2, 0, 0, 41, 5, maxint, 26, 16, maxint, 7, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, 4, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12 },
	{ 0, 0, 18, 17, 16, 0, 13, 0, 10, 0, 0, 0, 6, 0, 3, 0, 0, 0, 0, 2, 3, 4, 6, 0, 8, 28, 9, 0, 11, 0, 0, 14, 16, 0, 18, 0, 19, 26, 24, 0, 17, 19, 24, maxint, 9, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 30, 3, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 3, maxint, maxint, maxint, maxint, 15, maxint, 15, maxint, maxint, maxint, maxint, 8, maxint, 15, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 9, 8, 7, 0, 4, 0, 1, 0, 0, 0, 3, 0, 6, 0, 0, 0, 0, 11, 12, 13, 15, 0, 17, 19, 18, 0, 20, 0, 0, 23, 25, 0, 27, 0, 28, 17, 15, 0, 26, 10, 15, 9, maxint, maxint, 8, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 20, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, 13, 7, 16, maxint, maxint, maxint, 8, maxint, 11, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 8, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 12, 11, 10, 0, 7, 0, 4, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 8, 9, 10, 12, 0, 14, 22, 15, 0, 17, 0, 0, 20, 22, 0, 24, 0, 25, 20, 18, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 17, 11, 22, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 22, maxint, maxint, maxint },
	{ 0, 0, 1, 0, 1, 0, 4, 0, 7, 0, 0, 0, 11, 0, 14, 0, 0, 0, 0, 19, 20, 21, 23, 0, 25, 11, 26, 0, 28, 0, 0, 31, 33, 0, 35, 0, 36, 9, 7, 0, maxint, maxint, 7, maxint, maxint, 11, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, 28, maxint, maxint, maxint, 5, 8, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 7, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 28, maxint, maxint, maxint, 7, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 29, 28, 27, 0, 24, 0, 21, 0, 0, 0, 17, 0, 14, 0, 0, 0, 0, 9, 8, 7, 5, 0, 3, 39, 2, 0, 0, 0, 0, 3, 5, 0, 7, 0, 8, 37, 35, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 35, 29, 18, 41, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 29, maxint, maxint, 29, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 29, maxint, 29, maxint, maxint, maxint, maxint, maxint, maxint, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 3, 4, 5, 0, 8, 0, 11, 0, 0, 0, 15, 0, 18, 0, 0, 0, 0, 23, 24, 25, 27, 0, 29, 7, 30, 0, 32, 0, 0, 35, 37, 0, 39, 0, 40, 5, 3, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 32, maxint, 5, 16, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 38, maxint, 38, maxint, maxint, maxint, maxint, maxint, maxint, 38, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 2, 1, 0, 0, 3, 0, 6, 0, 0, 0, 10, 0, 13, 0, 0, 0, 0, 18, 19, 20, 22, 0, 24, 12, 25, 0, 27, 0, 0, 30, 32, 0, 34, 0, 35, 10, 8, 0, maxint, maxint, maxint, maxint, 7, 10, maxint, 27, 5, maxint, 11, 12, 7, maxint, maxint, maxint, maxint, maxint, maxint, 7, 13, 34, 29, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 24, maxint, maxint, maxint, maxint, maxint, maxint, 24, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, maxint, maxint, maxint, 24, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 12, 11, 10, 0, 7, 0, 4, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 8, 9, 10, 12, 0, 14, 22, 15, 0, 17, 0, 0, 20, 22, 0, 24, 0, 25, 20, 18, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 17, 15, 10, maxint, 23, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 17, 11, 22, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 22, maxint, maxint, maxint },
	{ 0, 0, 9, 10, 11, 0, 14, 0, 17, 0, 0, 0, 21, 0, 24, 0, 0, 0, 0, 29, 30, 31, 33, 0, 35, 1, 36, 0, 38, 0, 0, 41, 43, 0, 45, 0, 46, 1, 3, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 38, 6, 11, 21, maxint, maxint, 15, maxint, maxint, maxint, 15, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 15, maxint, maxint, maxint, maxint, maxint, 14, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 14, maxint, maxint, maxint, 15 },
	{ 0, 0, 9, 8, 7, 0, 4, 0, 1, 0, 0, 0, 3, 0, 6, 0, 0, 0, 0, 11, 12, 13, 15, 0, 17, 19, 18, 0, 20, 0, 0, 23, 25, 0, 27, 0, 28, 17, 15, 0, maxint, 10, maxint, maxint, maxint, maxint, 8, maxint, maxint, maxint, maxint, maxint, maxint, 4, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 20, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, 13, 7, 16, maxint, maxint, maxint, 8, maxint, 11, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 8, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 5, 4, 3, 0, 0, 0, 3, 0, 0, 0, 7, 0, 10, 0, 0, 0, 0, 15, 16, 17, 19, 0, 21, 15, 22, 0, 24, 0, 0, 27, 29, 0, 31, 0, 32, 13, 11, 0, maxint, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 4, maxint, maxint, 35, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, 2, 20, 15, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 35, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 5, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 27, 26, 25, 0, 22, 0, 19, 0, 0, 0, 15, 0, 12, 0, 0, 0, 0, 7, 6, 5, 3, 0, 1, 37, 0, 0, 2, 0, 0, 5, 7, 0, 9, 0, 10, 35, 33, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, 7, 24, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 6, 3, 28, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 29, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 37, 36, 35, 0, 32, 0, 29, 0, 0, 0, 25, 0, 22, 0, 0, 0, 0, 17, 16, 15, 13, 0, 11, 47, 10, 0, 8, 0, 0, 5, 3, 0, 1, 0, 0, 45, 43, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 10, maxint, 3, 35, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 40, maxint, maxint, maxint, 42, 35 },
	{ 0, 0, 34, 33, 32, 0, 29, 0, 26, 0, 0, 0, 22, 0, 19, 0, 0, 0, 0, 14, 13, 12, 10, 0, 8, 44, 7, 0, 5, 0, 0, 2, 0, 0, 2, 0, 3, 42, 40, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 7, 3, maxint, 31, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 37, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 5, 4, 3, 0, 0, 0, 3, 0, 0, 0, 7, 0, 10, 0, 0, 0, 0, 15, 16, 17, 19, 0, 21, 15, 22, 0, 24, 0, 0, 27, 29, 0, 31, 0, 32, 13, 11, 0, maxint, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 22, 32, 29, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, 2, 20, 15, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 35, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 5, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 6, 5, 4, 0, 1, 0, 2, 0, 0, 0, 6, 0, 9, 0, 0, 0, 0, 14, 15, 16, 18, 0, 20, 16, 21, 0, 23, 0, 0, 26, 28, 0, 30, 0, 31, 14, 12, 0, maxint, maxint, maxint, maxint, maxint, maxint, 5, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 3, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 5, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 26, 5, maxint, maxint, maxint, maxint, maxint, maxint, 26, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 9, 8, 7, 0, 4, 0, 1, 0, 0, 0, 3, 0, 6, 0, 0, 0, 0, 11, 12, 13, 15, 0, 17, 19, 18, 0, 20, 0, 0, 23, 25, 0, 27, 0, 28, 17, 15, 0, maxint, 10, maxint, maxint, maxint, maxint, 8, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 3, maxint, maxint, maxint, maxint, maxint, 20, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, 13, 7, 16, maxint, maxint, maxint, 8, maxint, 11, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 8, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 14, 13, 12, 0, 9, 0, 6, 0, 0, 0, 2, 0, 1, 0, 0, 0, 0, 6, 7, 8, 10, 0, 12, 24, 13, 0, 15, 0, 0, 18, 20, 0, 22, 0, 23, 22, 20, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 20, 16, maxint, maxint, 7, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 7, 13, maxint, maxint, 13, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 13, maxint, 13, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 33, 32, 31, 0, 28, 0, 25, 0, 0, 0, 21, 0, 18, 0, 0, 0, 0, 13, 12, 11, 9, 0, 7, 43, 6, 0, 4, 0, 0, 1, 1, 0, 3, 0, 4, 41, 39, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 31, maxint, maxint, maxint, maxint, 6, maxint, maxint, maxint, maxint, maxint, 19, maxint, 4, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 34, maxint, maxint, 34, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 6, maxint, maxint, 34, maxint, 34, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 29, 28, 27, 0, 24, 0, 21, 0, 0, 0, 17, 0, 14, 0, 0, 0, 0, 9, 8, 7, 5, 0, 3, 39, 2, 0, 0, 0, 0, 3, 5, 0, 7, 0, 8, 37, 35, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 27, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 15, 4, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 29, maxint, maxint, 29, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 29, maxint, 29, maxint, maxint, maxint, maxint, maxint, maxint, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 7, 8, 9, 0, 12, 0, 15, 0, 0, 0, 19, 0, 22, 0, 0, 0, 0, 27, 28, 29, 31, 0, 33, 3, 34, 0, 36, 0, 0, 39, 41, 0, 43, 0, 44, 1, 1, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 3, 30, 2, 47, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint },
	{ 0, 0, 10, 11, 12, 0, 15, 0, 18, 0, 0, 0, 22, 0, 25, 0, 0, 0, 0, 30, 31, 32, 34, 0, 36, 0, 37, 0, 39, 0, 0, 42, 44, 0, 46, 0, 47, 2, 4, 0, 45, 9, 4, 28, 19, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 3, maxint, 34, 1, 50, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 15, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 15, maxint, maxint, maxint, maxint },
	{ 0, 0, 21, 20, 19, 0, 16, 0, 13, 0, 0, 0, 9, 0, 6, 0, 0, 0, 0, 1, 0, 1, 3, 0, 5, 31, 6, 0, 8, 0, 0, 11, 13, 0, 15, 0, 16, 29, 27, 0, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, 28, 31, maxint, 33, 16, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 18, 3, 25, 7, 3, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 18, maxint, maxint, maxint, maxint },
	{ 0, 0, 9, 10, 11, 0, 14, 0, 17, 0, 0, 0, 21, 0, 24, 0, 0, 0, 0, 29, 30, 31, 33, 0, 35, 1, 36, 0, 38, 0, 0, 41, 43, 0, 45, 0, 46, 1, 3, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 14, maxint, maxint, maxint, 14, maxint, maxint, maxint, maxint, maxint, 2, 1, 30, maxint, 49, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 15, maxint, maxint, maxint, maxint, maxint, 14, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 14, maxint, maxint, maxint, 15 },
	{ 0, 0, 36, 35, 34, 0, 31, 0, 28, 0, 0, 0, 24, 0, 21, 0, 0, 0, 0, 16, 15, 14, 12, 0, 10, 46, 9, 0, 7, 0, 0, 4, 2, 0, 0, 0, 1, 44, 42, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 43, 46, 15, 45, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 30, maxint, maxint, maxint, maxint, maxint, maxint, 35, maxint, maxint, maxint, maxint, maxint, maxint, 30, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 35, 48, 1, maxint, maxint },
	{ 0, 0, 6, 7, 8, 0, 11, 0, 14, 0, 0, 0, 18, 0, 21, 0, 0, 0, 0, 26, 27, 28, 30, 0, 32, 4, 33, 0, 35, 0, 0, 38, 40, 0, 42, 0, 43, 2, 0, 0, maxint, maxint, maxint, maxint, maxint, maxint, 7, maxint, maxint, maxint, maxint, maxint, maxint, 11, maxint, maxint, maxint, 11, maxint, maxint, maxint, maxint, maxint, maxint, 4, maxint, maxint, maxint, maxint, 14, 33, 3, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12 },
	{ 0, 0, 7, 6, 5, 0, 2, 0, 1, 0, 0, 0, 5, 0, 8, 0, 0, 0, 0, 13, 14, 15, 17, 0, 19, 17, 20, 0, 22, 0, 0, 25, 27, 0, 29, 0, 30, 15, 13, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 2, maxint, maxint, maxint, 2, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 13, maxint, 18, 17, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 1, 2, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 1, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 25, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 25, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 2 },
	{ 0, 0, 24, 23, 22, 0, 19, 0, 16, 0, 0, 0, 12, 0, 9, 0, 0, 0, 0, 4, 3, 2, 0, 0, 2, 34, 3, 0, 5, 0, 0, 8, 10, 0, 12, 0, 13, 32, 30, 0, maxint, maxint, maxint, maxint, 15, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 15, 19, maxint, maxint, maxint, 19, maxint, 15, maxint, maxint, maxint, maxint, maxint, 3, maxint, maxint, 30, 17, maxint, 36, maxint, maxint, maxint, 3, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 20, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 24, 30, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 20 },
	{ 0, 0, 9, 10, 11, 0, 14, 0, 17, 0, 0, 0, 21, 0, 24, 0, 0, 0, 0, 29, 30, 31, 33, 0, 35, 1, 36, 0, 38, 0, 0, 41, 43, 0, 45, 0, 46, 1, 3, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 14, maxint, maxint, maxint, 14, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 3, 16, 33, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 15, maxint, maxint, maxint, maxint, maxint, 14, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 14, maxint, maxint, maxint, 15 },
	{ 0, 0, 36, 35, 34, 0, 31, 0, 28, 0, 0, 0, 24, 0, 21, 0, 0, 0, 0, 16, 15, 14, 12, 0, 10, 46, 9, 0, 7, 0, 0, 4, 2, 0, 0, 0, 1, 44, 42, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 31, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 30, maxint, maxint, maxint, maxint, maxint, maxint, 35, maxint, maxint, maxint, maxint, maxint, maxint, 30, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 35, 48, 1, maxint, maxint },
	{ 0, 0, 7, 6, 5, 0, 2, 0, 1, 0, 0, 0, 5, 0, 8, 0, 0, 0, 0, 13, 14, 15, 17, 0, 19, 17, 20, 0, 22, 0, 0, 25, 27, 0, 29, 0, 30, 15, 13, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 2, maxint, maxint, maxint, 2, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 29, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 1, 2, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 1, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 25, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 25, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 2 },
	{ 0, 0, 2, 3, 4, 0, 7, 0, 10, 0, 0, 0, 14, 0, 17, 0, 0, 0, 0, 22, 23, 24, 26, 0, 28, 8, 29, 0, 31, 0, 0, 34, 36, 0, 38, 0, 39, 6, 4, 0, maxint, maxint, maxint, maxint, 11, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, maxint, maxint, 39, maxint, maxint, maxint, 11, maxint, maxint, maxint, maxint, maxint, 23, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 25, 4, 28, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 42, maxint, 35, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 35, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 21, 20, 19, 0, 16, 0, 13, 0, 0, 0, 9, 0, 6, 0, 0, 0, 0, 1, 0, 1, 3, 0, 5, 31, 6, 0, 8, 0, 0, 11, 13, 0, 15, 0, 16, 29, 27, 0, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 23, maxint, 20, 3, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 18, 3, 25, 7, 3, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 18, maxint, maxint, maxint, maxint },
	{ 0, 0, 2, 1, 0, 0, 3, 0, 6, 0, 0, 0, 10, 0, 13, 0, 0, 0, 0, 18, 19, 20, 22, 0, 24, 12, 25, 0, 27, 0, 0, 30, 32, 0, 34, 0, 35, 10, 8, 0, maxint, maxint, maxint, maxint, 7, 10, maxint, maxint, maxint, maxint, 10, maxint, 7, maxint, maxint, maxint, maxint, maxint, maxint, 7, 12, 31, 27, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 22, maxint, maxint, maxint, 4, 19, maxint, 24, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, maxint, maxint, maxint, 24, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 24, 23, 22, 0, 19, 0, 16, 0, 0, 0, 12, 0, 9, 0, 0, 0, 0, 4, 3, 2, 0, 0, 2, 34, 3, 0, 5, 0, 0, 8, 10, 0, 12, 0, 13, 32, 30, 0, maxint, maxint, maxint, maxint, 15, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 15, 19, maxint, maxint, maxint, 19, maxint, 15, maxint, maxint, maxint, maxint, maxint, 3, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 26, 3, 22, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 20, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 24, 30, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 20 },
	{ 0, 0, 4, 5, 6, 0, 9, 0, 12, 0, 0, 0, 16, 0, 19, 0, 0, 0, 0, 24, 25, 26, 28, 0, 30, 6, 31, 0, 33, 0, 0, 36, 38, 0, 40, 0, 41, 4, 2, 0, maxint, maxint, maxint, maxint, maxint, 16, maxint, maxint, maxint, maxint, 16, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 28, maxint, maxint, maxint, maxint, maxint, maxint, 28, maxint, 6, 39, 5, maxint, maxint, maxint, maxint, maxint, maxint, 17, maxint, maxint, maxint, 30, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 17, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 2, 1, 0, 0, 3, 0, 6, 0, 0, 0, 10, 0, 13, 0, 0, 0, 0, 18, 19, 20, 22, 0, 24, 12, 25, 0, 27, 0, 0, 30, 32, 0, 34, 0, 35, 10, 8, 0, maxint, maxint, maxint, maxint, 7, 10, maxint, maxint, maxint, maxint, 10, maxint, 7, maxint, maxint, maxint, maxint, maxint, maxint, 7, 12, 31, 27, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 22, maxint, maxint, maxint, maxint, maxint, maxint, 22, 6, maxint, 33, 1, maxint, maxint, maxint, maxint, maxint, maxint, 11, maxint, maxint, maxint, 24, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 32, 31, 30, 0, 27, 0, 24, 0, 0, 0, 20, 0, 17, 0, 0, 0, 0, 12, 11, 10, 8, 0, 6, 42, 5, 0, 3, 0, 0, 0, 2, 0, 4, 0, 5, 40, 38, 0, maxint, maxint, maxint, maxint, maxint, 20, maxint, maxint, maxint, maxint, 20, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 36, 30, maxint, 34, maxint, maxint, maxint, maxint, maxint, maxint, 22, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 22, maxint, maxint, maxint, 38, 15, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 1, 0, 1, 0, 4, 0, 7, 0, 0, 0, 11, 0, 14, 0, 0, 0, 0, 19, 20, 21, 23, 0, 25, 11, 26, 0, 28, 0, 0, 31, 33, 0, 35, 0, 36, 9, 7, 0, maxint, maxint, 7, maxint, maxint, 11, maxint, maxint, maxint, maxint, 11, maxint, maxint, maxint, 26, maxint, maxint, maxint, 5, 8, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 7, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 5, 1, 31, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 28, maxint, maxint, maxint, 7, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 32, 31, 30, 0, 27, 0, 24, 0, 0, 0, 20, 0, 17, 0, 0, 0, 0, 12, 11, 10, 8, 0, 6, 42, 5, 0, 3, 0, 0, 0, 2, 0, 4, 0, 5, 40, 38, 0, maxint, maxint, maxint, maxint, maxint, 20, maxint, maxint, maxint, maxint, 20, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 36, 26, maxint, maxint, maxint, 22, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 22, maxint, maxint, maxint, 38, 15, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 1, 2, 3, 0, 6, 0, 9, 0, 0, 0, 13, 0, 16, 0, 0, 0, 0, 21, 22, 23, 25, 0, 27, 9, 28, 0, 30, 0, 0, 33, 35, 0, 37, 0, 38, 7, 5, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 10, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 9, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 33, maxint, 9, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 8, 7, 6, 0, 3, 0, 0, 0, 0, 0, 4, 0, 7, 0, 0, 0, 0, 12, 13, 14, 16, 0, 18, 18, 19, 0, 21, 0, 0, 24, 26, 0, 28, 0, 29, 16, 14, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 28, 1, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 24, 9, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 5, 4, 3, 0, 0, 0, 3, 0, 0, 0, 7, 0, 10, 0, 0, 0, 0, 15, 16, 17, 19, 0, 21, 15, 22, 0, 24, 0, 0, 27, 29, 0, 31, 0, 32, 13, 11, 0, maxint, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 32, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, 2, 19, 14, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 6, maxint, maxint, 9, 23, 7, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 35, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 5, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 14, 13, 12, 0, 9, 0, 6, 0, 0, 0, 2, 0, 1, 0, 0, 0, 0, 6, 7, 8, 10, 0, 12, 24, 13, 0, 15, 0, 0, 18, 20, 0, 22, 0, 23, 22, 20, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 7, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 7, 12, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, 9, maxint, 13, 2, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 13, maxint, 13, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 26, 25, 24, 0, 21, 0, 18, 0, 0, 0, 14, 0, 11, 0, 0, 0, 0, 6, 5, 4, 2, 0, 0, 36, 1, 0, 3, 0, 0, 6, 8, 0, 10, 0, 11, 34, 32, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 21, 12, maxint, 15, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 8, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 12, 11, 10, 0, 7, 0, 4, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 8, 9, 10, 12, 0, 14, 22, 15, 0, 17, 0, 0, 20, 22, 0, 24, 0, 25, 20, 18, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 16, 10, 20, 11, maxint, maxint, maxint, 7, 2, 14, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 22, maxint, maxint, maxint },
	{ 0, 0, 32, 31, 30, 0, 27, 0, 24, 0, 0, 0, 20, 0, 17, 0, 0, 0, 0, 12, 11, 10, 8, 0, 6, 42, 5, 0, 3, 0, 0, 0, 2, 0, 4, 0, 5, 40, 38, 0, maxint, maxint, maxint, maxint, maxint, 20, maxint, maxint, maxint, maxint, 20, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 20, maxint, 39, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 22, maxint, maxint, maxint, 38, 15, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 4, 5, 6, 0, 9, 0, 12, 0, 0, 0, 16, 0, 19, 0, 0, 0, 0, 24, 25, 26, 28, 0, 30, 6, 31, 0, 33, 0, 0, 36, 38, 0, 40, 0, 41, 4, 2, 0, maxint, maxint, maxint, maxint, maxint, 16, maxint, maxint, maxint, maxint, 16, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 28, maxint, maxint, maxint, maxint, maxint, maxint, 28, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 16, 36, maxint, maxint, 30, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 17, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 4, 3, 2, 0, 1, 0, 4, 0, 0, 0, 8, 0, 11, 0, 0, 0, 0, 16, 17, 18, 20, 0, 22, 14, 23, 0, 25, 0, 0, 28, 30, 0, 32, 0, 33, 12, 10, 0, 31, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, 14, 17, 13, 32, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 34, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 34, maxint, maxint },
	{ 0, 0, 24, 23, 22, 0, 19, 0, 16, 0, 0, 0, 12, 0, 9, 0, 0, 0, 0, 4, 3, 2, 0, 0, 2, 34, 3, 0, 5, 0, 0, 8, 10, 0, 12, 0, 13, 32, 30, 0, maxint, maxint, maxint, maxint, 15, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 15, 19, maxint, maxint, maxint, 19, maxint, 15, maxint, maxint, maxint, maxint, maxint, 3, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 3, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 19, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 28, 11, 6, 3, maxint, maxint, maxint, maxint, 24, 30, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 20 },
	{ 0, 0, 2, 3, 4, 0, 7, 0, 10, 0, 0, 0, 14, 0, 17, 0, 0, 0, 0, 22, 23, 24, 26, 0, 28, 8, 29, 0, 31, 0, 0, 34, 36, 0, 38, 0, 39, 6, 4, 0, maxint, maxint, maxint, maxint, 11, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, maxint, maxint, 39, maxint, maxint, maxint, 11, maxint, maxint, maxint, maxint, maxint, 23, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 23, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 26, maxint, 17, 22, 31, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 42, maxint, 35, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 35, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 14, 13, 12, 0, 9, 0, 6, 0, 0, 0, 2, 0, 1, 0, 0, 0, 0, 6, 7, 8, 10, 0, 12, 24, 13, 0, 15, 0, 0, 18, 20, 0, 22, 0, 23, 22, 20, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 7, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 7, 12, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 10, 16, maxint, 4, 14, maxint, maxint, 13, maxint, 13, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 18, 17, 16, 0, 13, 0, 10, 0, 0, 0, 6, 0, 3, 0, 0, 0, 0, 2, 3, 4, 6, 0, 8, 28, 9, 0, 11, 0, 0, 14, 16, 0, 18, 0, 19, 26, 24, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 28, 3, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 3, maxint, maxint, maxint, maxint, 14, maxint, 14, maxint, maxint, maxint, maxint, 8, maxint, 14, maxint, maxint, 6, 20, 4, maxint, 9, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 27, 26, 25, 0, 22, 0, 19, 0, 0, 0, 15, 0, 12, 0, 0, 0, 0, 7, 6, 5, 3, 0, 1, 37, 0, 0, 2, 0, 0, 5, 7, 0, 9, 0, 10, 35, 33, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 3, 29, 13, 9, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 6, 3, 28, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 29, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 35, 34, 33, 0, 30, 0, 27, 0, 0, 0, 23, 0, 20, 0, 0, 0, 0, 15, 14, 13, 11, 0, 9, 45, 8, 0, 6, 0, 0, 3, 1, 0, 1, 0, 2, 43, 41, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 45, maxint, maxint, 1, maxint, maxint, maxint, maxint, 1, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 29, 36, 45, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 34, maxint, maxint, maxint, maxint },
	{ 0, 0, 8, 7, 6, 0, 3, 0, 0, 0, 0, 0, 4, 0, 7, 0, 0, 0, 0, 12, 13, 14, 16, 0, 18, 18, 19, 0, 21, 0, 0, 24, 26, 0, 28, 0, 29, 16, 14, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 28, 1, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 27, maxint, 6, 15, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 2, 1, 0, 0, 3, 0, 6, 0, 0, 0, 10, 0, 13, 0, 0, 0, 0, 18, 19, 20, 22, 0, 24, 12, 25, 0, 27, 0, 0, 30, 32, 0, 34, 0, 35, 10, 8, 0, maxint, maxint, maxint, maxint, 7, 10, maxint, maxint, maxint, maxint, 10, maxint, 7, maxint, maxint, maxint, maxint, maxint, maxint, 7, 12, 31, 27, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 22, maxint, maxint, maxint, maxint, maxint, maxint, 22, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 10, maxint, maxint, maxint, 22, maxint, maxint, maxint, maxint, 33, 6, maxint, 8, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 6, 7, 8, 0, 11, 0, 14, 0, 0, 0, 18, 0, 21, 0, 0, 0, 0, 26, 27, 28, 30, 0, 32, 4, 33, 0, 35, 0, 0, 38, 40, 0, 42, 0, 43, 2, 0, 0, maxint, maxint, maxint, maxint, maxint, maxint, 7, maxint, maxint, maxint, maxint, maxint, maxint, 11, maxint, maxint, maxint, 11, maxint, maxint, maxint, maxint, maxint, maxint, 4, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 41, 14, 8, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 12 },
	{ 0, 0, 2, 1, 0, 0, 3, 0, 6, 0, 0, 0, 10, 0, 13, 0, 0, 0, 0, 18, 19, 20, 22, 0, 24, 12, 25, 0, 27, 0, 0, 30, 32, 0, 34, 0, 35, 10, 8, 0, maxint, maxint, maxint, maxint, 7, 10, maxint, maxint, maxint, maxint, 10, maxint, 7, maxint, maxint, maxint, maxint, maxint, maxint, 7, 12, 31, 27, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 22, maxint, maxint, maxint, maxint, maxint, maxint, 22, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 10, maxint, maxint, maxint, 22, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 4, 5, 6, 0, 9, 0, 12, 0, 0, 0, 16, 0, 19, 0, 0, 0, 0, 24, 25, 26, 28, 0, 30, 6, 31, 0, 33, 0, 0, 36, 38, 0, 40, 0, 41, 4, 2, 0, maxint, maxint, maxint, maxint, maxint, 16, maxint, maxint, maxint, maxint, 16, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 28, maxint, maxint, maxint, maxint, maxint, maxint, 28, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 16, maxint, maxint, maxint, 28, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 17, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 18, 17, 16, 0, 13, 0, 10, 0, 0, 0, 6, 0, 3, 0, 0, 0, 0, 2, 3, 4, 6, 0, 8, 28, 9, 0, 11, 0, 0, 14, 16, 0, 18, 0, 19, 26, 24, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 28, 3, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 3, maxint, maxint, maxint, maxint, 14, maxint, 14, maxint, maxint, maxint, maxint, 8, maxint, 14, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 20, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 37, 36, 35, 0, 32, 0, 29, 0, 0, 0, 25, 0, 22, 0, 0, 0, 0, 17, 16, 15, 13, 0, 11, 47, 10, 0, 8, 0, 0, 5, 3, 0, 1, 0, 0, 45, 43, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 19, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 40, maxint, maxint, maxint, 42, 35 },
	{ 0, 0, 33, 32, 31, 0, 28, 0, 25, 0, 0, 0, 21, 0, 18, 0, 0, 0, 0, 13, 12, 11, 9, 0, 7, 43, 6, 0, 4, 0, 0, 1, 1, 0, 3, 0, 4, 41, 39, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 31, maxint, maxint, maxint, maxint, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 31, maxint, maxint, 31, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 6, maxint, maxint, 31, maxint, 31, maxint, maxint, maxint, maxint, 3, 35, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 30, 29, 28, 0, 25, 0, 22, 0, 0, 0, 18, 0, 15, 0, 0, 0, 0, 10, 9, 8, 6, 0, 4, 40, 3, 0, 1, 0, 0, 2, 4, 0, 6, 0, 7, 38, 36, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 3, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 3, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 3, maxint, 31, maxint, 25, 2, 35, 26, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 1, 0, 1, 0, 4, 0, 7, 0, 0, 0, 11, 0, 14, 0, 0, 0, 0, 19, 20, 21, 23, 0, 25, 11, 26, 0, 28, 0, 0, 31, 33, 0, 35, 0, 36, 9, 7, 0, maxint, maxint, 7, maxint, maxint, 11, maxint, maxint, maxint, maxint, 11, maxint, maxint, maxint, 26, maxint, maxint, maxint, 5, 8, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 7, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 26, maxint, maxint, maxint, 7, maxint, maxint, maxint, maxint, 32, 29, maxint, maxint, maxint, maxint, maxint, maxint, 12, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 29, 28, 27, 0, 24, 0, 21, 0, 0, 0, 17, 0, 14, 0, 0, 0, 0, 9, 8, 7, 5, 0, 3, 39, 2, 0, 0, 0, 0, 3, 5, 0, 7, 0, 8, 37, 35, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 27, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 27, maxint, maxint, 27, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 27, maxint, 27, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 7, 6, 5, 0, 2, 0, 1, 0, 0, 0, 5, 0, 8, 0, 0, 0, 0, 13, 14, 15, 17, 0, 19, 17, 20, 0, 22, 0, 0, 25, 27, 0, 29, 0, 30, 15, 13, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 2, maxint, maxint, maxint, 2, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 1, 2, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 1, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 23, maxint, maxint, maxint, 23, 9, 1, maxint, 25, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 2 },
	{ 0, 0, 28, 27, 26, 0, 23, 0, 20, 0, 0, 0, 16, 0, 13, 0, 0, 0, 0, 8, 7, 6, 4, 0, 2, 38, 1, 0, 1, 0, 0, 4, 6, 0, 8, 0, 9, 36, 34, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 2, maxint, maxint, 21, maxint, 33, 24, maxint, 2, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 2, 3, 4, 0, 7, 0, 10, 0, 0, 0, 14, 0, 17, 0, 0, 0, 0, 22, 23, 24, 26, 0, 28, 8, 29, 0, 31, 0, 0, 34, 36, 0, 38, 0, 39, 6, 4, 0, maxint, maxint, maxint, maxint, 11, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, maxint, maxint, 39, maxint, maxint, maxint, 11, maxint, maxint, maxint, maxint, maxint, 23, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 23, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 39, maxint, 32, maxint, maxint, 9, 30, maxint, 8, maxint, 35, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 6, 5, 4, 0, 1, 0, 2, 0, 0, 0, 6, 0, 9, 0, 0, 0, 0, 14, 15, 16, 18, 0, 20, 16, 21, 0, 23, 0, 0, 26, 28, 0, 30, 0, 31, 14, 12, 0, maxint, maxint, maxint, maxint, maxint, maxint, 5, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 5, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 24, 5, maxint, 1, 22, 8, maxint, maxint, 26, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 12, 11, 10, 0, 7, 0, 4, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 8, 9, 10, 12, 0, 14, 22, 15, 0, 17, 0, 0, 20, 22, 0, 24, 0, 25, 20, 18, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 16, 10, 20, 11, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 19, 23, 6, maxint, maxint, maxint, maxint, 22, maxint, maxint, maxint },
	{ 0, 0, 30, 29, 28, 0, 25, 0, 22, 0, 0, 0, 18, 0, 15, 0, 0, 0, 0, 10, 9, 8, 6, 0, 4, 40, 3, 0, 1, 0, 0, 2, 4, 0, 6, 0, 7, 38, 36, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 3, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 3, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 23, 2, 32, 24, 18, maxint, 3, 26, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 33, 32, 31, 0, 28, 0, 25, 0, 0, 0, 21, 0, 18, 0, 0, 0, 0, 13, 12, 11, 9, 0, 7, 43, 6, 0, 4, 0, 0, 1, 1, 0, 3, 0, 4, 41, 39, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 31, maxint, maxint, maxint, maxint, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 31, maxint, maxint, 31, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 6, maxint, maxint, 31, maxint, 31, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 21, 3, maxint, 29, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 6, 5, 4, 0, 1, 0, 2, 0, 0, 0, 6, 0, 9, 0, 0, 0, 0, 14, 15, 16, 18, 0, 20, 16, 21, 0, 23, 0, 0, 26, 28, 0, 30, 0, 31, 14, 12, 0, maxint, maxint, maxint, maxint, maxint, maxint, 5, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 5, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 24, 5, maxint, maxint, maxint, maxint, maxint, 6, 24, 27, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 3, 4, 5, 0, 8, 0, 11, 0, 0, 0, 15, 0, 18, 0, 0, 0, 0, 23, 24, 25, 27, 0, 29, 7, 30, 0, 32, 0, 0, 35, 37, 0, 39, 0, 40, 5, 3, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 35, maxint, 35, maxint, maxint, maxint, maxint, maxint, maxint, 35, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 23, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 18, 17, 16, 0, 13, 0, 10, 0, 0, 0, 6, 0, 3, 0, 0, 0, 0, 2, 3, 4, 6, 0, 8, 28, 9, 0, 11, 0, 0, 14, 16, 0, 18, 0, 19, 26, 24, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 28, 3, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 3, maxint, maxint, maxint, maxint, 14, maxint, 14, maxint, maxint, maxint, maxint, 8, maxint, 14, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 21, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 0, 1, 2, 0, 5, 0, 8, 0, 0, 0, 12, 0, 15, 0, 0, 0, 0, 20, 21, 22, 24, 0, 26, 10, 27, 0, 29, 0, 0, 32, 34, 0, 36, 0, 37, 8, 6, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 27, 37, 34, 5, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint },
	{ 0, 0, 4, 3, 2, 0, 1, 0, 4, 0, 0, 0, 8, 0, 11, 0, 0, 0, 0, 16, 17, 18, 20, 0, 22, 14, 23, 0, 25, 0, 0, 28, 30, 0, 32, 0, 33, 12, 10, 0, 31, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, 14, 17, 13, 32, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 31, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 34, maxint, maxint },
	{ 0, 0, 8, 9, 10, 0, 13, 0, 16, 0, 0, 0, 20, 0, 23, 0, 0, 0, 0, 28, 29, 30, 32, 0, 34, 2, 35, 0, 37, 0, 0, 40, 42, 0, 44, 0, 45, 0, 2, 0, maxint, maxint, maxint, maxint, maxint, 20, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 44, maxint, maxint, maxint, maxint, 44, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 47, maxint, maxint },
	{ 0, 0, 35, 34, 33, 0, 30, 0, 27, 0, 0, 0, 23, 0, 20, 0, 0, 0, 0, 15, 14, 13, 11, 0, 9, 45, 8, 0, 6, 0, 0, 3, 1, 0, 1, 0, 2, 43, 41, 0, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 45, maxint, maxint, 1, maxint, maxint, maxint, maxint, 1, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 31, 43, maxint, maxint, maxint },
	{ 0, 0, 2, 3, 4, 0, 7, 0, 10, 0, 0, 0, 14, 0, 17, 0, 0, 0, 0, 22, 23, 24, 26, 0, 28, 8, 29, 0, 31, 0, 0, 34, 36, 0, 38, 0, 39, 6, 4, 0, maxint, maxint, maxint, maxint, 11, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, maxint, maxint, 39, maxint, maxint, maxint, 11, maxint, maxint, maxint, maxint, maxint, 23, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 23, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 39, maxint, 32, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 32, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 7 },
	{ 0, 0, 5, 4, 3, 0, 0, 0, 3, 0, 0, 0, 7, 0, 10, 0, 0, 0, 0, 15, 16, 17, 19, 0, 21, 15, 22, 0, 24, 0, 0, 27, 29, 0, 31, 0, 32, 13, 11, 0, maxint, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 32, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 11, 2, 19, 14, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 6, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 32, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, maxint, 5, maxint, maxint, maxint, 7, maxint },
};

int num_e[20] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 }; // elevator numbers (fi)

int Size_l = 20;
int Size_r = 115;
int Size_r2 = 21;
int Size_Elevator_Requests = 9;
int Size_d = 85;

bool d_visited[85] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
int d_waitingtime[85] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
int d_travelingtime[85] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
int d_stops[85] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
int TotalCost = 0;

bool flag_print = false;
int Round = 1;

int num_elevators = m;

bool flag_split = true; /* merge and split until flag is false */


/* structure for edges including source , destinations and type of them*/

struct Edges
{
	int source;
	int destination;
	int weight;
	int source_type; /* 1 for elevator, 2 for requests and 3 for destinations */
	int source_request_type;
	int destination_type; /* 1 for elevator ( are requests! and requests), 2 for destinations */
};


bool edgescompare(Edges lhs, Edges rhs)
{
	return lhs.weight <= rhs.weight;
}

struct coalition
{
	int elements[1000];
	int types[1000];
	double value;
	int number;
	bool reviewed;
};

struct history
{
	int elevators[100];
	int number;
};
coalition* Coalition = new coalition[Size_l];

queue<coalition> myqueue; /*  SS */
std::vector<coalition> myvector;


/* beginning of Kruskal MST*/

// Creating shortcut for an integer pair 
typedef  pair<int, int> iPair;

// Structure to represent a graph 
struct Graph
{
	int V, E;
	vector< pair<int, iPair> > edges;

	// Constructor 
	Graph(int V, int E)
	{
		this->V = V;
		this->E = E;
	}

	// Utility function to add an edge 
	void addEdge(int u, int v, int w)
	{
		edges.push_back({ w,{ u, v } });
	}

	// Function to find MST using Kruskal's 
	// MST algorithm 
	int kruskalMST(int** graph);
};

// To represent Disjoint Sets 
struct DisjointSets
{
	int *parent, *rnk;
	int n;

	// Constructor. 
	DisjointSets(int n)
	{
		// Allocate memory 
		this->n = n;
		parent = new int[n + 1];
		rnk = new int[n + 1];

		// Initially, all vertices are in 
		// different sets and have rank 0. 
		for (int i = 0; i <= n; i++)
		{
			rnk[i] = 0;

			//every element is parent of itself 
			parent[i] = i;
		}
	}

	// Find the parent of a node 'u' 
	// Path Compression 
	int find(int u)
	{
		/* Make the parent of the nodes in the path
		from u--> parent[u] point to parent[u] */
		if (u != parent[u])
			parent[u] = find(parent[u]);
		return parent[u];
	}

	// Union by rank 
	void merge(int x, int y)
	{
		x = find(x), y = find(y);

		/* Make tree with smaller height
		a subtree of the other tree  */
		if (rnk[x] > rnk[y])
			parent[y] = x;
		else // If rnk[x] <= rnk[y] 
			parent[x] = y;

		if (rnk[x] == rnk[y])
			rnk[y]++;
	}
};

/* Functions returns weight of the MST*/

int Graph::kruskalMST(int** graph_new)
{
	int mst_wt = 0; // Initialize result 

	// Sort edges in increasing order on basis of cost 
	sort(edges.begin(), edges.end());

	// Create disjoint sets 
	DisjointSets ds(V);

	// Iterate through all sorted edges 
	vector< pair<int, iPair> >::iterator it;
	for (it = edges.begin(); it != edges.end(); it++)
	{
		int u = it->second.first;
		int v = it->second.second;

		int set_u = ds.find(u);
		int set_v = ds.find(v);

		// Check if the selected edge is creating 
		// a cycle or not (Cycle is created if u 
		// and v belong to same set) 
		if (set_u != set_v)
		{
			// Current edge will be in the MST 

			/* initialization of MST sources and destinations*/
			MST_sources[MST_count] = u;
			MST_destinations[MST_count] = v;
			MST_count++;

			// Update MST weight 
			mst_wt += it->first;

			// Merge two sets 
			ds.merge(set_u, set_v);
		}
	}

	return mst_wt;
}


/* end of Kruskal MST*/
int IP(coalition s, bool *VisitedNodes, int *Indices, int **graph_new, int Count_Nodes)
{

	int value;
	IloEnv env;
	try {
		IloModel model(env);


		IloArray<IloBoolVarArray> x(env, Count_Nodes);

		for (int i = 0; i < Count_Nodes; ++i)
		{
			x[i] = IloBoolVarArray(env, Count_Nodes);
		}
		IloIntVarArray h(env, Count_Nodes);

		for (int i = 0;i<Count_Nodes;i++)
			h[i] = IloIntVar(env, 1, Count_Nodes); // subtour elimination h[i] refers to reuqest r[i]


		IloIntVarArray u(env, Count_Nodes);

		for (int i = 0;i<Count_Nodes;i++)
			u[i] = IloIntVar(env, 1, Count_Nodes); // It denotes the elevator origin (serve the requests in its serving path) 


		IloExpr expression(env);

		for (int i = 0;i<Count_Nodes;i++)
			for (int j = 0;j < Count_Nodes;j++)
			{
				bool flag_i = false; // not in L
				bool flag_j = false; // not in L

				bool flag_ii = false; // not in S
				bool flag_jj = false; // not in S

				for (int t = 0;t<Size_l;t++)
					if (Indices[i] == l[t])
						flag_i = true;

				for (int t = 0;t<Size_l;t++)
					if (Indices[j] == l[t])
						flag_j = true;

				if (!flag_i && !flag_j)
					expression += graph_new[i][j] * x[i][j];
				else
				{
					for (int t = 0;t<s.number;t++)
						if (Indices[i] == s.elements[t])
							flag_ii = true;
					for (int t = 0;t<s.number;t++)
						if (Indices[j] == s.elements[t])
							flag_jj = true;

					if (flag_i && flag_ii && flag_j && flag_jj)
						expression += graph_new[i][j] * x[i][j];
					else if (flag_i && flag_ii && !flag_j)
						expression += graph_new[i][j] * x[i][j];
					else if (flag_j && flag_jj && !flag_i)
						expression += graph_new[i][j] * x[i][j];
				}

			}


		model.add(IloMinimize(env, expression)); // minimize the cost

		//constraints


		IloExpr Expr(env);

		for (int i = 0;i < s.number;i++)
		{
			int i1 = s.elements[i];
			bool flag_i = false;
			int index_i = -1;
			for (int m = 0;m<Count_Nodes;m++)
				if (Indices[m] == i1)
				{
					flag_i = true;
					index_i = m;
				}
			for (int j = 0;j < Size_r;j++)
			{
				int j1 = r[j];
				bool flag_j = false;
				int index_j = -1;
				for (int m = 0;m<Count_Nodes;m++)
					if (Indices[m] == j1)
					{
						flag_j = true;
						index_j = m;
					}

				if (flag_i && flag_j)
					Expr += x[index_i][index_j]; // location i,j are i-1 and j-1 in matrix
			}

			int index;

			for (int t = 0;t<Size_l;t++)
				if (l[t] == s.elements[i])
				{
					index = t;
				}

			if (flag_i)
				model.add(Expr <= num_e[index]);
			Expr.end();
			Expr = IloExpr(env);
		}

		IloExpr Expr2(env);

		for (int i = 0;i < s.number;i++)
		{
			int i1 = s.elements[i];
			bool flag_i = false;
			int index_i = -1;
			for (int m = 0;m<Count_Nodes;m++)
				if (Indices[m] == i1)
				{
					flag_i = true;
					index_i = m;
				}
			for (int j = 0;j < Size_r;j++)
			{
				int j1 = r[j];
				bool flag_j = false;
				int index_j = -1;
				for (int m = 0;m<Count_Nodes;m++)
					if (Indices[m] == j1)
					{
						flag_j = true;
						index_j = m;
					}
				if (flag_i && flag_j)
					Expr2 += x[index_j][index_i];
			}

			int index;

			for (int t = 0;t<Size_l;t++)
				if (l[t] == s.elements[i])
				{
					index = t;
				}
			if (flag_i)
				model.add(Expr2 <= num_e[index]);
			Expr2.end();
			Expr2 = IloExpr(env);
		}

		
		IloExpr Expr3(env);

		for (int j = 0;j < Size_r2 + Size_Elevator_Requests;j++)
		{
			int j1 = AllRequests[j];
			bool flag_j = false;
			int index_j = -1;
			for (int m = 0;m<Count_Nodes;m++)
				if (Indices[m] == j1)
				{
					flag_j = true;
					index_j = m;
				}
			for (int i = 0;i < n;i++)
			{
				bool flag_i = false; // not in L
				bool flag_ii = false; // not in S

				for (int t = 0;t<Size_l;t++)
					if (i == l[t] - 1)
						flag_i = true;

				bool flag_i_validity = false;
				int index_i_validity = -1;
				for (int m = 0;m<Count_Nodes;m++)
					if (Indices[m] == i + 1)
					{
						flag_i_validity = true;
						index_i_validity = m;
					}

				if (!flag_i && flag_i_validity && flag_j)
					Expr3 += x[index_i_validity][index_j];
				else
				{
					for (int t = 0;t<s.number;t++)
						if (i == s.elements[t] - 1)
							flag_ii = true;

					if (flag_i && flag_ii && flag_i_validity && flag_j)
						Expr3 += x[index_i_validity][index_j];
				}
			}
			if (flag_j)
				model.add(Expr3 == 1);

			Expr3.end();
			Expr3 = IloExpr(env);
		}

		IloExpr Expr4(env);
		for (int j = 0;j<Size_d;j++)
		{
			int j1 = d[j];
			bool flag_j = false;
			int index_j = -1;
			for (int m = 0;m<Count_Nodes;m++)
				if (Indices[m] == j1)
				{
					flag_j = true;
					index_j = m;
				}
			for (int i = 0;i < n;i++)
			{
				bool flag_i = false; // not in L
				bool flag_ii = false; // not in S

				for (int t = 0;t<Size_l;t++)
					if (i == l[t] - 1)
						flag_i = true;

				bool flag_i_validity = false;
				int  index_i_validity = -1;
				for (int m = 0;m<Count_Nodes;m++)
					if (Indices[m] == i + 1)
					{
						flag_i_validity = true;
						index_i_validity = m;
					}

				if (!flag_i && flag_i_validity && flag_j)
					Expr4 += x[index_i_validity][index_j];
				else
				{
					for (int t = 0;t<s.number;t++)
						if (i == s.elements[t] - 1)
							flag_ii = true;

					if (flag_i && flag_ii && flag_i_validity && flag_j)
						Expr4 += x[index_i_validity][index_j];
				}
			}
			if (flag_j)
				model.add(Expr4 == 1);
			Expr4.end();
			Expr4 = IloExpr(env);
		}

		IloExpr Expr5(env);

		for (int j = 0;j < Size_d;j++)
		{
			int j1 = d[j];
			bool flag_j = false;
			int index_j = -1;
			for (int m = 0;m<Count_Nodes;m++)
				if (Indices[m] == j1)
				{
					flag_j = true;
					index_j = m;
				}
			for (int i = 0;i < n;i++)
			{
				bool flag_i = false; // not in L
				bool flag_ii = false; // not in S

				for (int t = 0;t<Size_l;t++)
					if (i == l[t] - 1)
						flag_i = true;

				bool flag_i_validity = false;
				int  index_i_validity = -1;
				for (int m = 0;m<Count_Nodes;m++)
					if (Indices[m] == i + 1)
					{
						flag_i_validity = true;
						index_i_validity = m;
					}

				if (!flag_i && flag_i_validity && flag_j)
					Expr5 += x[index_j][index_i_validity];
				else
				{
					for (int t = 0;t<s.number;t++)
						if (i == s.elements[t] - 1)
							flag_ii = true;

					if (flag_i && flag_ii && flag_i_validity && flag_j)
						Expr5 += x[index_j][index_i_validity];
				}
			}
			if (flag_j)
				model.add(Expr5 == 1);
			Expr5.end();
			Expr5 = IloExpr(env);
		}

		IloExpr Expr6(env);
		IloExpr Expr7(env);

		for (int j = 0;j < Size_r;j++)
		{
			int j1 = r[j];
			bool flag_j = false;
			int index_j = -1;
			for (int m = 0;m<Count_Nodes;m++)
				if (Indices[m] == j1)
				{
					flag_j = true;
					index_j = m;
				}
			for (int i = 0;i < n;i++)
			{
				bool flag_i = false; // not in L
				bool flag_ii = false; // not in S

				for (int t = 0;t<Size_l;t++)
					if (i == l[t] - 1)
						flag_i = true;
				bool flag_i_validity = false;
				int index_i_validity = -1;
				for (int m = 0;m<Count_Nodes;m++)
					if (Indices[m] == i + 1)
					{
						flag_i_validity = true;
						index_i_validity = m;
					}

				if (!flag_i && flag_i_validity && flag_j)
				{
					Expr6 += x[index_i_validity][index_j];
					Expr7 += x[index_j][index_i_validity];
				}
				else
				{
					for (int t = 0;t<s.number;t++)
						if (i == s.elements[t] - 1)
							flag_ii = true;

					if (flag_i && flag_ii && flag_i_validity && flag_j)
					{
						Expr6 += x[index_i_validity][index_j];
						Expr7 += x[index_j][index_i_validity];
					}
				}
			}
			if (flag_j)
				model.add(Expr6 == Expr7);
			Expr6.end();
			Expr7.end();
			Expr6 = IloExpr(env);
			Expr7 = IloExpr(env);
		}


		IloExpr Expr9(env);

		for (int i = 0;i<Size_l;i++)
		{
			bool flag = false;
			int index = l[i];
			int index_Indices = -1;
			for (int k = 0;k < Count_Nodes;k++)
				if (Indices[k] == index)
				{
					index_Indices = k;
					flag = true;
				}

			if (flag)
				model.add(u[index_Indices] - (index_Indices + 1) == 0);

			Expr9.end();
			Expr9 = IloExpr(env);
		}

		IloExpr Expr10(env);

		for (int i = 0;i<Count_Nodes;i++)
			for (int j = 0;j < Count_Nodes;j++)
			{
				bool flag = false;
				for (int k = 0;k<Size_l;k++)
					if (l[k] == Indices[j])
						flag = true;
				if (!flag)
				{
					model.add(u[i] - u[j] + x[i][j] * U_Max <= U_Max);
					model.add(u[j] - u[i] + x[i][j] * U_Max <= U_Max);

				}

				Expr10.end();
				Expr10 = IloExpr(env);
			}

		IloExpr Expr8(env);

		for (int i = 0;i<Size_r;i++)
			for (int j = 0;j < Size_r;j++)
			{
				int i1 = r[i];
				int j1 = r[j];

				bool flag_i = false;
				int index_i = -1;
				for (int m = 0;m<Count_Nodes;m++)
					if (Indices[m] == i1)
					{
						flag_i = true;
						index_i = m;
					}
				bool flag_j = false;
				int index_j = -1;
				for (int m = 0;m<Count_Nodes;m++)
					if (Indices[m] == j1)
					{
						flag_j = true;
						index_j = m;
					}

				if (i != j && index_i != index_j && flag_i && flag_j)
				{
					model.add(h[index_i] - h[index_j] + p*x[index_i][index_j] <= p - 1);

				}
				if (i >= Size_r2 + Size_Elevator_Requests)
				{
					int index_i = 0;

					bool flag = false;

					for (int m = 0;m<Size_r2 + Size_Elevator_Requests;m++)
						if (AllRequests_Sources[m] == D[i - Size_r2 - Size_Elevator_Requests])
						{
							index_i = AllRequests[m];
							flag = true;
						}

					int i1 = d[i - Size_r2 - Size_Elevator_Requests];

					bool flag_i_validity = false;  // for index_i 
					int index_i_validity = -1;
					for (int m = 0;m<Count_Nodes;m++)
						if (Indices[m] == index_i)
						{
							flag_i_validity = true;
							index_i_validity = m;
						}

					bool flag_i1_validity = false;  // for i1 
					int index_i1_validity = -1;
					for (int m = 0;m<Count_Nodes;m++)
						if (Indices[m] == i1)
						{
							flag_i1_validity = true;
							index_i1_validity = m;
						}

					if (flag_i_validity && flag_i1_validity)
						model.add((h[index_i_validity] - h[index_i1_validity] + 1 <= 0) && (u[index_i_validity] - u[index_i1_validity] == 0));

				}
				Expr8.end();
				Expr8 = IloExpr(env);
			}

		//solving

		IloCplex cplex(env);

		cplex.setParam(IloCplex::Param::TimeLimit, 360);

		cplex.setOut(env.getNullStream()); // omitir para problemas grandas para ver como avanza la solucion
		cplex.extract(model);
		cplex.exportModel("model.lp");
		env.out() << "Variables binarias: " << cplex.getNbinVars() << endl;
		env.out() << "Variables Enteras: " << cplex.getNintVars() << endl;
		env.out() << "Filas - Restricciones: " << cplex.getNrows() << endl;
		env.out() << "Columnas - Variables: " << cplex.getNcols() << endl;

		if (!cplex.solve()) {
			env.error() << "No es posible resolver :-(" << endl;
			throw (-1);
		}

		env.out() << "Es optimo ? = " << cplex.getStatus() << endl;
		env.out() << "Valor de fo = " << cplex.getObjValue() << endl;
		env.out() << "Se demoro   = " << env.getTime() << endl;

		value = cplex.getObjValue();

		for (int i = 0;i<Count_Nodes;i++)
			for (int j = 0;j<Count_Nodes;j++)
				if (cplex.getValue(x[i][j]) == 1)
				{
					int index_i = Indices[i];
					int index_j = Indices[j];

					bool elevator_check = false;
					int label = -1;
					for (int k = 0;k < Size_l; k++)
						if (index_j == l[k])
						{
							elevator_check = true;
							label = cplex.getValue(u[i]);
						}
					if (elevator_check)
					{
						for (int m = 0;m < Size_l;m++)
						{

							for (int z = 0;z < Count_Nodes;z++)
								if (Indices[z] == l[m])
									if (label == cplex.getValue(u[z]))
										for (int n = 0;n < Size_d;n++)
											if (d[n] == index_i)
											{

												env.out() << " Elevator " << l[m] << " moves from " << E[m] << " to " << DF[n] << endl;
												E[m] = DF[n];
												env.out() << " Elevator " << l[m] << " new location " << E[m] << endl;
											}

						}
					}

					env.out() << "x[" << index_i << "][" << index_j << "]=" << cplex.getValue(x[i][j]) << "  " << graph_new[i][j] << endl;

				}

		for (int i = 0;i < Count_Nodes;i++)
		{
			bool flag_u = false;
			for (int j = 0;j<Size_r;j++)
				if (r[j] == Indices[i])
					flag_u = true;

			if (flag_u)
				env.out() << "h[" << Indices[i] << "]" << cplex.getValue(h[i]) << endl;
		}

		for (int i = 0;i < Count_Nodes;i++)
		{

			env.out() << "u[" << Indices[i] << "]" << cplex.getValue(u[i]) << endl;
		}
	}
	catch (IloException& ex) {
		cerr << "Error Cplex: " << ex << endl;
	}
	catch (...) {
		cerr << "Error Cpp" << endl;
	}

	env.end();

	return value;
}

void MergeCoalitions()
{
	bool repeat = false;

	while (!repeat)
	{
		int num_unreviewed = 0;
		for (unsigned i = 0; i<myvector.size(); ++i)
		{
			coalition temp = myvector[i];
			if (temp.reviewed == false)
				num_unreviewed++;
		}

		if (num_unreviewed<2)
		{
			repeat = true; /* break the merge loop */
		}
		int unreviewed_service1, unreviewed_service2;
		if (num_unreviewed >= 2)
		{
			bool flag = false;
			srand(time(NULL));
			while (!flag)
			{
				unreviewed_service1 = rand() % num_unreviewed + 1;
				unreviewed_service2 = rand() % num_unreviewed + 1;
				if (unreviewed_service1 != unreviewed_service2)
					flag = true;
			}
		}

		coalition S1;
		coalition S2;

		int index1 = 0;
		int index2 = 0;

		if (num_unreviewed >= 2)
		{
			int counter = 0;
			for (unsigned i = 0; i<myvector.size(); ++i)
			{
				coalition temp = myvector[i];
				if (temp.reviewed == false)
					counter++;
				if (counter == unreviewed_service1)
				{
					S1 = temp;
					break;
				}
				else
					index1++;
			}
			counter = 0;
			for (unsigned i = 0; i<myvector.size(); ++i)
			{
				coalition temp = myvector[i];
				if (temp.reviewed == false)
					counter++;
				if (counter == unreviewed_service2)
				{
					S2 = temp;
					break;
				}
				else
					index2++;
			}
		}

		/* the end of selecting two unreviewed service coalitions */
		coalition temp;
		if (num_unreviewed >= 2)
		{
			temp.number = S1.number + S2.number;

			for (int i = 0;i<S1.number;i++)
				temp.elements[i] = S1.elements[i];
			for (int i = 0;i<S2.number;i++)
				temp.elements[i + S1.number] = S2.elements[i];
			temp.value = 0; /* initial value */
			temp.reviewed = false;

			double profit_temp;
			double profit_S1;
			double profit_S2;
			if (temp.number >= ceil(n1 / p1))
				profit_temp = 1 / ((1 + temp.value)*temp.number);
			else
				profit_temp = 0;

			if (S1.number >= ceil(n1 / p1))
				profit_S1 = 1 / ((1 + S1.value)*S1.number);
			else
				profit_S1 = 0;

			if (S2.number >= ceil(n1 / p1))
				profit_S2 = 1 / ((1 + S2.value)*S2.number);
			else
				profit_S2 = 0;


			if ((profit_temp >= profit_S1) && (profit_temp >= profit_S2))
			{

				coalition temp1 = myvector[0 + index1];
				coalition temp2 = myvector[0 + index2];

				if (index1 >= index2)
				{
					myvector.erase(myvector.begin() + index1);
					myvector.erase(myvector.begin() + index2);
				}
				else
				{
					myvector.erase(myvector.begin() + index2);
					myvector.erase(myvector.begin() + index1);
				}

				myvector.push_back(temp);
			}
			else
			{
				coalition temp1 = myvector[0 + index1];
				temp1.reviewed = true;
				coalition temp2 = myvector[0 + index2];
				temp2.reviewed = true;
				if (index1 >= index2)
				{
					myvector.erase(myvector.begin() + index1);
					myvector.erase(myvector.begin() + index2);
				}
				else
				{
					myvector.erase(myvector.begin() + index2);
					myvector.erase(myvector.begin() + index1);
				}

				myvector.push_back(temp1);

				myvector.push_back(temp2);

			}
		}
	}
}
void SplitCoalition()
{
	flag_split = false;
	srand(time(NULL));
	for (int i1 = 0; i1<myvector.size(); i1++)
	{
		coalition Si = myvector[i1];

		if (Si.number >= ceil(n1 / p1))
		{
			for (int i = 1;i<pow(2, Si.number) - 2;i++)
			{
				coalition Sj;
				Sj.number = 0;
				coalition Sk;
				Sk.number = 0;
				for (int j = 0;j<Si.number;j++)
				{

					if ((i >> j) % 2 == 1)
					{
						Sj.elements[Sj.number] = Si.elements[j];
						Sj.number++;
					}
					else
					{
						Sk.elements[Sk.number] = Si.elements[j];
						Sk.number++;
					}
				}
				Sj.reviewed = false;
				Sk.reviewed = false;

				if (Sj.number >= ceil(n1 / p1) || Sk.number >= ceil(n1 / p1))
				{

					if ((1 / ((1 + Sj.value)*Sj.number) >= 1 / ((1 + Si.value)*Si.number) && Sj.number >= ceil(n1 / p1)) || (1 / ((1 + Sk.value)*Sk.number) >= 1 / ((1 + Si.value)*Si.number) && Sk.number >= ceil(n1 / p1)))
					{
						myvector.erase(myvector.begin() + i1);
						myvector.push_back(Sj);
						myvector.push_back(Sk);
						flag_split = true;
						break;
					}
				}
			}

		}
	}
}

/*starting of DFS */

class Graph1
{
	int V; // No. of vertices 

    // Pointer to an array containing 
    // adjacency lists 
	list<int> *adj;

	// A recursive function used by DFS 
	void DFSUtil(int v, bool visited[], int parent, int* arr, int size);
public:
	Graph1(int V); // Constructor 

	// function to add an edge to graph 
	void addEdge(int v, int w);

	// DFS traversal of the vertices 
	// reachable from v 
	void DFS(int v, int* arr, int size);
};

Graph1::Graph1(int V)
{
	this->V = V;
	adj = new list<int>[V];
}

void Graph1::addEdge(int v, int w)
{
	adj[v].push_back(w); // Add w to vӳ list. 
}

void Graph1::DFSUtil(int v, bool visited[], int parent, int* arr, int size)
{
	// Mark the current node as visited and 
	visited[v] = true;
	// Recur for all the vertices adjacent 
	// to this vertex 
	list<int>::iterator i;
	for (i = adj[v].begin(); i != adj[v].end(); ++i)
		if (!visited[*i])
		{
			/* CHECK to see if *i is a destination */
			bool checking_d = false;
			int source_d = -1;
			for (int m = 0;m<Size_d;m++)
				if (d[m] == ((*i) + 1))
				{
					checking_d = true; source_d = D[m];
				}
			/* End of checking */
			int source = -1;
			for (int m = 0;m<Size_l;m++)
				if (l[m] == (v + 1))
				{
					source = E[m];
				}
			for (int m = 0;m<Size_d;m++)
				if (d[m] == (v + 1))
				{
					source = D[m];
				}
			for (int m = 0;m<Size_r2;m++)
				if (r2[m] == (v + 1))
				{
					source = R[m];
				}
			/* finding the source floor */

			/* end of checking for finding source floor */
			bool flag = false;
			for (int j = 0;j<size;j++)
				if (arr[j] == (*i) + 1)
					flag = true;
			if (!flag)
				if (!checking_d || (checking_d && source == source_d))
					DFSUtil(*i, visited, v, arr, size);
		}

}

// DFS traversal of the vertices reachable from v. 
// It uses recursive DFSUtil() 
void Graph1::DFS(int v, int* arr, int size)
{
	// Mark all the vertices as not visited 
	bool *visited = new bool[V];
	for (int i = 0; i < V; i++)
		visited[i] = false;

	// Call the recursive helper function 
	// to print DFS traversal 
	DFSUtil(v, visited, -1, arr, size);
}


/* end of DFS */


Edges* Greedy(Edges* edges, int NumofEdges, int NumNodes, int NumElevators)
{

	int* source_keepers = new int[NumNodes];
	bool* visited = new bool[NumofEdges];

	int* VisitedNodes = new int[NumNodes];
	int* VisitedNodes_Incoming = new int[NumNodes];
	int* VisitedNodes_Outgoing = new int[NumNodes];

	for (int i = 0;i<NumNodes;i++)
	{
		VisitedNodes[i] = -1;
		VisitedNodes_Incoming[i] = 0;
		VisitedNodes_Outgoing[i] = 0;
	}

	Edges* SelectedEdges = new Edges[n - 1];  

	int Source_Numbers = 0;

	for (int i = 0;i<NumofEdges;i++)
		visited[i] = false;

	int total_weight = 0;

	int count_edges = 0;
	int prev_count_edges = 0;
	int next_count_edges = 0;
	/* 3 conditions: a) not cycle b) not two outgoing or incoming c) floors before destinations */
	bool while_flag = true;
	int elevator_index = -1;

	int last_edge = 0;

	while (while_flag)
	{
		prev_count_edges = next_count_edges;

		for (int i = 0;i<NumofEdges;i++)
			if (!visited[i])
			{
				int source = edges[i].source;
				int dest = edges[i].destination;

				bool flag_validity = true;

				/* checking incoming and outgoing edges */
				for (int z = 0;z<NumNodes;z++)
					if (VisitedNodes[z] == source)
						if (VisitedNodes_Outgoing[z] == 1)
							flag_validity = false;
				for (int z = 0;z<NumNodes;z++)
					if (VisitedNodes[z] == dest)
						if (VisitedNodes_Incoming[z] == 1)
							flag_validity = false;
				/* checking for a cycle! */
				bool flag_source = false;
				for (int z = 0;z<NumNodes;z++)
					if (VisitedNodes[z] == source)
						flag_source = true;
				bool flag_dest = false;
				for (int z = 0;z<NumNodes;z++)
					if (VisitedNodes[z] == dest)
						flag_dest = true;
				if (flag_source && flag_dest)
					flag_validity = false;


				if (edges[i].source_type == 1 && flag_validity) /* source is an elevator */
				{
					for (int j = 0;j<Size_l;j++)
						if (source + 1 == l[j])
						{
							source_keepers[Source_Numbers++] = E[j];
							elevator_index = j;
						}

					SelectedEdges[count_edges] = edges[i];
					if (flag_print)
						cout << edges[i].source + 1 << " -> " << edges[i].destination + 1 << " : " << edges[i].weight << endl;

					last_edge = edges[i].destination + 1;

					count_edges++;
					next_count_edges = count_edges;

					visited[i] = true;

					/* for source node */
					bool flag_enter_source = false;
					int index1 = -1;
					for (int z = 0;z < NumNodes;z++)
						if (VisitedNodes[z] == source)
						{
							flag_enter_source = true;
							index1 = z;
						}
					if (flag_enter_source)
					{
						VisitedNodes_Outgoing[index1]++;
					}
					else for (int z = 0;z < NumNodes;z++)
						if (VisitedNodes[z] == -1)
						{
							VisitedNodes[z] = source;
							VisitedNodes_Outgoing[z]++;
							z = NumNodes;
						}
					/* for destination node */
					bool flag_enter_dest = false;
					int index2 = -1;
					for (int z = 0;z < NumNodes;z++)
						if (VisitedNodes[z] == dest)
						{
							flag_enter_dest = true;
							index2 = z;
						}
					if (flag_enter_dest)
					{
						VisitedNodes_Incoming[index2]++;
					}
					else for (int z = 0;z < NumNodes;z++)
						if (VisitedNodes[z] == -1)
						{
							VisitedNodes[z] = dest;
							VisitedNodes_Incoming[z]++;
							z = NumNodes;
						}


					i = NumofEdges;

				}
				if (edges[i].source_type == 2 && flag_validity) /* source is a request */
				{
					bool flag_request = false;
					int index3 = -1;
					for (int z = 0;z<NumNodes;z++)
						if (VisitedNodes[z] == source)
						{
							flag_request = true;
							index3 = z;
						}
					if (index3 >= 0)
						if (flag_request && VisitedNodes_Incoming[index3] > 0)
						{
							int request_index = -1;
							int elevator_request_index = -1;
							bool flag_elevator_request = false;

							for (int j = 0;j < Size_r2;j++)
								if (source + 1 == r2[j])
								{
									source_keepers[Source_Numbers++] = R[j];
									request_index = j;
								}

							for (int j = 0;j < Size_Elevator_Requests;j++)
								if (source + 1 == Elevator_Requests[j])
								{
									source_keepers[Source_Numbers++] = Elevator_Requests_Sources[j];
								}


							SelectedEdges[count_edges] = edges[i];


							if (flag_print)
								cout << edges[i].source + 1 << " -> " << edges[i].destination + 1 << " : " << edges[i].weight << endl;

							last_edge = edges[i].destination + 1;

							count_edges++;
							next_count_edges = count_edges;
							visited[i] = true;

							/* for source node */
							bool flag_enter_source = false;
							int index1 = -1;
							for (int z = 0;z < NumNodes;z++)
								if (VisitedNodes[z] == source)
								{
									flag_enter_source = true;
									index1 = z;
								}
							if (flag_enter_source)
							{
								VisitedNodes_Outgoing[index1]++;
							}
							else for (int z = 0;z < NumNodes;z++)
								if (VisitedNodes[z] == -1)
								{
									VisitedNodes[z] = source;
									VisitedNodes_Outgoing[z]++;
									z = NumNodes;
								}
							/* for destination node */
							bool flag_enter_dest = false;
							int index2 = -1;
							for (int z = 0;z < NumNodes;z++)
								if (VisitedNodes[z] == dest)
								{
									flag_enter_dest = true;
									index2 = z;
								}
							if (flag_enter_dest)
							{
								VisitedNodes_Incoming[index2]++;
							}
							else for (int z = 0;z < NumNodes;z++)
								if (VisitedNodes[z] == -1)
								{
									VisitedNodes[z] = dest;
									VisitedNodes_Incoming[z]++;
									z = NumNodes;
								}

							i = NumofEdges;

						}
				}
				if (edges[i].source_type == 3 && flag_validity) /* source is a destination */
				{

					int source_floor = -1;
					for (int j = 0;j<Size_d;j++)
						if (d[j] == source + 1)
							source_floor = D[j];
					bool flag = false;
					for (int k = 0;k<Source_Numbers;k++)
						if (source_keepers[k] == source_floor)
							flag = true;

					bool flag_temp = false;
					for (int z = 0;z<NumNodes;z++)
						if (VisitedNodes[z] == source)
						{
							flag_temp = true;
							z = NumNodes;
						}
					if (!flag_temp)
						flag = false;

					bool flag2 = false;
					if (edges[i].destination_type == 3)
					{
						int source_floor_d = -1;
						for (int j = 0;j<Size_d;j++)
							if (d[j] == dest + 1)
								source_floor_d = D[j];
						for (int k = 0;k<Source_Numbers;k++)
							if (source_keepers[k] == source_floor_d)
								flag2 = true;
					}
					else
						flag2 = true;

					if (flag && flag2) /* add to the selected edges and visited[i] = true but first check circle or an eulerion constraint */
					{
						SelectedEdges[count_edges] = edges[i];
						if (flag_print)
							cout << edges[i].source + 1 << " -> " << edges[i].destination + 1 << " : " << edges[i].weight << endl;

						last_edge = edges[i].destination + 1;

						count_edges++;
						next_count_edges = count_edges;
						visited[i] = true;
						i = NumofEdges;
						/* for source node */
						bool flag_enter_source = false;
						int index1 = -1;
						for (int z = 0;z<NumNodes;z++)
							if (VisitedNodes[z] == source)
							{
								flag_enter_source = true;
								index1 = z;
							}
						if (flag_enter_source)
						{
							VisitedNodes_Outgoing[index1]++;
						}
						else for (int z = 0;z<NumNodes;z++)
							if (VisitedNodes[z] == -1)
							{
								VisitedNodes[z] = source;
								VisitedNodes_Outgoing[z]++;
								z = NumNodes;
							}
						/* for destination node */
						bool flag_enter_dest = false;
						int index2 = -1;
						for (int z = 0;z<NumNodes;z++)
							if (VisitedNodes[z] == dest)
							{
								flag_enter_dest = true;
								index2 = z;
							}
						if (flag_enter_dest)
						{
							VisitedNodes_Incoming[index2]++;
						}
						else for (int z = 0;z<NumNodes;z++)
							if (VisitedNodes[z] == -1)
							{
								VisitedNodes[z] = dest;
								VisitedNodes_Incoming[z]++;
								z = NumNodes;
							}

					}

				}

			}

		bool while_check = false;
		for (int z = 0;z<NumNodes;z++)
			if (VisitedNodes[z] == -1)
			{
				while_check = true;
			}
		if (!while_check)
			while_flag = false;

	}

	Num_SelectedEdges = count_edges;

	if (flag_round)
	{
		/* for checking if destination is an elevator and the source of that */
		int source_ei = -1;
		bool flag_ei = false;
		for (int z = 0;z < Size_l;z++)
		{
			if (last_edge == l[z])
			{
				flag_ei = true;
				source_ei = E[z];
			}
		}

		/* for checking if destination is a request and the source of that */
		int source_ri = -1;
		bool flag_ri = false;
		for (int z = 0;z < Size_r2;z++)
		{
			if (last_edge == r2[z])
			{
				flag_ri = true;
				source_ri = R[z];
			}
		}
		for (int z = 0;z < Size_Elevator_Requests;z++)
		{
			if (last_edge == Elevator_Requests[z])
			{
				flag_ri = true;
				source_ri = Elevator_Requests_Sources[z];
			}
		}

		/* for checking if edge's destination type is a destination and the source of that */
		int source_di = -1;
		int dest_di = -1;
		bool flag_di = false;
		for (int z = 0;z < Size_d;z++)
		{
			if (last_edge == d[z])
			{
				flag_di = true;
				source_di = D[z];
				dest_di = DF[z];
			}
		}
		/* end of checking */
		cout << " elevator's first location " << E[elevator_index] << "  ";
		if (flag_ei)
			E[elevator_index] = source_ei;
		if (flag_ri)
			E[elevator_index] = source_ri;
		if (flag_di)
			E[elevator_index] = dest_di;

		cout << " elevator's next location " << E[elevator_index] << " elevator's index in L " << elevator_index + 1 << endl;

	}


	return SelectedEdges;
}

Edges * BuildGraph(int elevator, int elevator_index, bool **graph, int **graph_new, int *existing, /*int *type,*/ int size, bool flag_graph)
{
	int* source_floors = new int[1 + size];
	for (int i = 0;i<1 + size;i++)
		source_floors[i] = -1;
	int source_elevator = -1;

	for (int k = 0;k<Size_l;k++) /* adding source floor of elevator in the coalition to the set */
		if (l[k] == elevator)
		{
			source_elevator = E[k];
		}
	int num = 0;
	for (int k = 0;k<Size_l;k++) /* adding source floor of elevator in the coalition to the set */
		if (l[k] == elevator)
		{
			source_floors[num++] = E[k]; check_l[k] = true;
			k = Size_l;
		}

	for (int z = 0;z < size;z++)
	{

		for (int k = 0;k<Size_r2;k++)
			if (r2[k] == existing[z])
			{
				source_floors[num++] = R[k];
				k = Size_r2;
			}
		for (int k = 0;k<Size_Elevator_Requests;k++)
			if (Elevator_Requests[k] == existing[z])
			{
				source_floors[num++] = Elevator_Requests_Sources[k];
				k = Size_Elevator_Requests;
			}
	}

	int NumofEdges = 0;
	for (int i = 0;i<n;i++) /* count number of valid edges in the graph */
		for (int j = 0;j<n;j++)
			if (graph[i][j])
			{
				int source_i = -1;
				bool desti_check = false;
				bool flag_ei = false;
				bool nodei_correctness = false;

				for (int m = 0;m<Size_l;m++)
					if (l[m] == (i + 1) && l[m] == elevator) 
					{
						source_i = E[m];
						flag_ei = true;
						nodei_correctness = true;
					}
				for (int m = 0;m<Size_d;m++)
					if (d[m] == (i + 1))
					{
						source_i = D[m];
						desti_check = true;

						for (int k = 1;k<size + 1;k++)
							if (source_floors[k] == source_i)
								nodei_correctness = true;
					}
				for (int m = 0;m<Size_r2;m++)
					if (r2[m] == (i + 1))
					{
						source_i = R[m];
						for (int z = 0; z < size; z++)
							if (existing[z] == r2[m])
							{
								nodei_correctness = true;
								z = size;
							}
					}
				for (int m = 0;m<Size_Elevator_Requests;m++)
					if (Elevator_Requests[m] == (i + 1))
					{
						source_i = Elevator_Requests_Sources[m];
						for (int z = 0; z < size; z++)
							if (existing[z] == Elevator_Requests[m])
							{
								nodei_correctness = true;
								z = size;
							}
					}

				int source_j = -1;
				bool destj_check = false;
				bool flag_j_check = false;
				bool flag_ej = false;
				bool nodej_correctness = true;

				for (int m = 0;m<Size_l;m++)
					if (l[m] == (j + 1) && l[m] == elevator)
					{
						source_j = E[m];
						flag_ej = true;
						nodej_correctness = true;
					}
				for (int m = 0;m<Size_d;m++)
					if (d[m] == (j + 1))
					{
						source_j = D[m];
						destj_check = true;
						for (int k = 1;k<size + 1;k++)
							if (source_floors[k] == source_j)
								nodej_correctness = true;
					}
				for (int m = 0;m<Size_r2;m++)
					if (r2[m] == (j + 1))
					{
						source_j = R[m];
						for (int z = 0; z < size; z++)
							if (existing[z] == r2[m])
							{
								nodej_correctness = true;
								z = size;
							}
					}
				for (int m = 0;m<Size_Elevator_Requests;m++)
					if (Elevator_Requests[m] == (j + 1))
					{
						source_j = Elevator_Requests_Sources[m];
						for (int z = 0; z < size; z++)
							if (existing[z] == Elevator_Requests[m])
							{
								nodej_correctness = true;
								z = size;
							}
					}

				bool flag_i = false;
				for (int k = 0;k<size + 1;k++)
					if (source_floors[k] == source_i && !(i + 1 != elevator && flag_ei))
						flag_i = true;
				bool flag_j = false;
				for (int k = 0;k<size + 1;k++)
					if (source_floors[k] == source_j && !(j + 1 != elevator && flag_ej))
						flag_j = true;
				if (flag_i && flag_j && nodei_correctness && nodej_correctness)
				{

					NumofEdges++;
				}

			}

	Edges *edges = new Edges[NumofEdges];

	int *VisitedNodes = new int[n];
	for (int k = 0;k<n;k++)
		VisitedNodes[k] = -1;

	int NumNodes = 0;
	int EdgesNum = 0;
	for (int i = 0;i<n;i++) /* count number of valid edges in the graph */
		for (int j = 0;j<n;j++)
			if (graph[i][j])
			{
				int source_i = -1;
				bool desti_check = false;
				bool flag_ei = false;
				bool nodei_correctness = false;

				for (int m = 0;m<Size_l;m++)
					if (l[m] == (i + 1) && l[m] == elevator)
					{
						flag_ei = true;
						source_i = E[m];
						nodei_correctness = true;
					}
				for (int m = 0;m<Size_d;m++)
					if (d[m] == (i + 1))
					{
						source_i = D[m];
						desti_check = true;
						for (int k = 1;k<size + 1;k++)
							if (source_floors[k] == source_i)
								nodei_correctness = true;
					}
				for (int m = 0;m<Size_r2;m++)
					if (r2[m] == (i + 1))
					{
						source_i = R[m];
						for (int z = 0; z < size; z++)
							if (existing[z] == r2[m])
							{
								nodei_correctness = true;
								z = size;
							}
					}
				for (int m = 0;m<Size_Elevator_Requests;m++)
					if (Elevator_Requests[m] == (i + 1))
					{
						source_i = Elevator_Requests_Sources[m];
						for (int z = 0; z < size; z++)
							if (existing[z] == Elevator_Requests[m])
							{
								nodei_correctness = true;
								z = size;
							}
					}

				int source_j = -1;
				bool destj_check = false;
				bool flag_ej = false;
				bool nodej_correctness = false;

				for (int m = 0;m<Size_l;m++)
					if (l[m] == (j + 1) && l[m] == elevator)
					{
						flag_ej = true;
						source_j = E[m];
						nodej_correctness = true;
					}
				for (int m = 0;m<Size_d;m++)
					if (d[m] == (j + 1))
					{
						source_j = D[m];
						destj_check = true;
						for (int k = 1;k<size + 1;k++)
							if (source_floors[k] == source_j)
								nodej_correctness = true;
					}
				for (int m = 0;m<Size_r2;m++)
					if (r2[m] == (j + 1))
					{
						source_j = R[m];
						for (int z = 0; z < size; z++)
							if (existing[z] == r2[m])
							{
								nodej_correctness = true;
								z = size;
							}
					}
				for (int m = 0;m<Size_Elevator_Requests;m++)
					if (Elevator_Requests[m] == (j + 1))
					{
						source_j = Elevator_Requests_Sources[m];
						for (int z = 0; z < size; z++)
							if (existing[z] == Elevator_Requests[m])
							{
								nodej_correctness = true;
								z = size;
							}
					}
				bool flag_i = false;
				for (int k = 0;k<size + 1;k++)
					if (source_floors[k] == source_i && !(i + 1 != elevator && flag_ei))
						flag_i = true;
				bool flag_j = false;
				for (int k = 0;k<size + 1;k++)
					if (source_floors[k] == source_j && !(j + 1 != elevator && flag_ej))
						flag_j = true;

				if (flag_i && flag_j && nodei_correctness && nodej_correctness)
				{

					edges[EdgesNum].source = i;
					edges[EdgesNum].destination = j;
					edges[EdgesNum].weight = graph_new[i][j];

					for (int k = 0;k<Size_l;k++)
						if (l[k] == i + 1)
						{
							bool flag = false;
							if (elevator == l[k])
								flag = true;

							if (flag)
								edges[EdgesNum].source_type = 1;
						}

					for (int k = 0;k<Size_r2;k++)
						if (r2[k] == i + 1)
							edges[EdgesNum].source_type = 2;

					for (int k = 0;k<Size_Elevator_Requests;k++)
						if (Elevator_Requests[k] == i + 1)
						{
							edges[EdgesNum].source_type = 2;
						}

					for (int k = 0;k<Size_d;k++)
						if (d[k] == i + 1)
							edges[EdgesNum].source_type = 3;

					for (int k = 0;k<Size_r2;k++)
						if (r2[k] == j + 1)
							edges[EdgesNum].destination_type = 2;
					for (int k = 0;k<Size_Elevator_Requests;k++)
						if (Elevator_Requests[k] == j + 1)
							edges[EdgesNum].destination_type = 2;
					for (int k = 0;k<Size_d;k++)
						if (d[k] == j + 1)
							edges[EdgesNum].destination_type = 3;
					EdgesNum++;

					/* count number of nodes */
					bool flag1 = false;
					for (int k = 0;k<n;k++)
						if (VisitedNodes[k] == i)
						{
							flag1 = true;
						}

					if (!flag1)
					{
						for (int k = 0;k<n;k++)
							if (VisitedNodes[k] == -1)
							{
								VisitedNodes[k] = i;
								NumNodes++;
								k = n;
							}
					}

					bool flag2 = false;
					for (int k = 0;k<n;k++)
						if (VisitedNodes[k] == j)
						{
							flag2 = true;
						}

					if (!flag2)
					{

						for (int k = 0;k<n;k++)
							if (VisitedNodes[k] == -1)
							{
								VisitedNodes[k] = j;
								NumNodes++;
								k = n;
							}
					}
					/* end of calculating NumNodes */
				}
			}
	std::sort(edges, edges + NumofEdges, edgescompare);
	Edges * output = Greedy(edges, NumofEdges, NumNodes, 1);

	return output;
}
void SCFM()
{


	for (int i = 1;i <= num_elevators;i++)
	{
		for (int j = 0;j < num_e[i - 1];j++)
		{

			coalition temp;
			temp.number = 1;
			temp.elements[0] = l[i - 1];
			temp.value = 0;
			temp.reviewed = false;
			myvector.push_back(temp);
		}

	}
	int counter = 0;
	while (flag_split)
	{
		MergeCoalitions();
		SplitCoalition();
		counter++;
	}
	coalition min = myvector[0];
	for (unsigned i = 0; i<myvector.size(); ++i)
	{
		coalition Si = myvector[i];
		if (Si.value < min.value)
			min = Si;
	}
	cout << "cost for whole coalition in min (S*)" << min.value << "  number of elevtors:  " << min.number << endl;

	cout << "result" << endl;
	for (int i = 0;i<min.number;i++)
		cout << min.elements[i] << " ";

	int Not_L = m - min.number;

	int* NL = new int[Not_L];
	int count = 0;

	for (int i = 0;i < m;i++)  /* finds the elevators not in the coalition */
	{
		bool flag = false;
		for (int j = 0;j<min.number;j++)
			if (l[i] == min.elements[j])
				flag = true;
		if (!flag)
		{
			NL[count++] = l[i];
			cout << "(i) " << l[i] << endl;
		}

	}

	int** graph_new = new int*[n];
	for (int i = 0; i < n; ++i)
		graph_new[i] = new int[n];

	bool** graph = new bool*[n];
	for (int i = 0; i < n; ++i)
		graph[i] = new bool[n];

	for (int i = 0; i < n; ++i)
		for (int j = 0; j < n; ++j)
			graph[i][j] = false;

	for (int i = 0; i < n; ++i)
		for (int j = 0; j < n; ++j)
			graph_new[i][j] = C[i][j];

	int i1 = -1;
	int j1 = 0;

	for (int i = 0;i < n;i++)
	{

		/* for checking if it is an elevator and the source of that */
		int source_ei = -1;
		bool flag_ei = false;
		for (int z = 0;z < Size_l;z++)
		{
			if (i + 1 == l[z])
			{
				flag_ei = true;
				source_ei = E[z];
			}
		}


		/* for checking if it is a request and the source of that */
		int source_ri = -1;
		bool flag_ri = false;
		for (int z = 0;z < Size_r2;z++)
		{
			if (i + 1 == r2[z])
			{
				flag_ri = true;
				source_ri = R[z];
			}
		}

		/* for checking if it is a destination and the source of that */
		int source_di = -1;
		int dest_di = -1;
		bool flag_di = false;
		for (int z = 0;z < Size_d;z++)
		{
			if (i + 1 == d[z])
			{
				flag_di = true;
				source_di = D[z];
				dest_di = DF[z];
			}
		}
		/* end of checking */
		j1 = 0;
		bool flag = false;
		for (int j = 0;j<Not_L;j++)
			if (i + 1 == NL[j])
				flag = true;
		if (!flag)
			i1++;
		for (int j = 0;j < n;j++)
		{

			/* for checking if it is a request and the source of that */
			int source_rj = -1;
			bool flag_rj = false;
			for (int z = 0;z < Size_r2;z++)
			{
				if (j + 1 == r2[z])
				{
					flag_rj = true;
					source_rj = R[z];
				}
			}

			/* for checking if it is a destination and the source of that */
			int source_dj = -1;
			int dest_dj = -1;
			bool flag_dj = false;
			for (int z = 0;z < Size_d;z++)
			{
				if (j + 1 == d[z])
				{
					flag_dj = true;
					source_dj = D[z];
					dest_dj = DF[z];
				}
			}

			/* end of checking */
			bool flag1 = false;
			for (int k = 0;k<Not_L;k++)
				if (j + 1 == NL[k])
					flag1 = true;
			if (!flag1 && !flag)
			{

				if (flag_ei && flag_rj)
					graph[i][j] = true;
				if (flag_ri && flag_dj && source_ri == source_dj)
					graph[i][j] = true;
				if (flag_di && flag_dj)
				{
					if (i != j)
					{
						graph[i][j] = true;
						graph[j][i] = true;
						graph_new[i][j] = abs(dest_di - dest_dj);
						graph_new[j][i] = abs(dest_di - dest_dj);
					}
				}
				if (flag_di && flag_rj && source_rj != source_di)
					graph[i][j] = true;
				if (flag_ei && flag_dj && source_ei == source_dj)
				{
					graph[i][j] = true;
				}
				if (flag_ri && flag_rj)
				{
					if (i != j)
					{
						graph[i][j] = true;
						graph[j][i] = true;
					}
				}
				j1++;

			}

			if (flag) /* elevator is not in the coalition */
			{
				if (flag_ei && flag_dj && source_ei == source_dj)
				{
					graph[i][j] = true;
					graph_new[i][j] = abs(dest_dj - source_ei);

					for (int z = 0;z < Size_l;z++)
					{
						if (l[z] != i + 1)
						{
							bool flag_no = false;
							for (int k = 0;k<Not_L;k++)
								if (l[z] == NL[k])
									flag_no = true;
							int loc = l[z];
							if (!flag_no)
							{
								graph[loc - 1][i] = true;
								graph_new[loc - 1][i] = abs(source_ei - E[z]);
							}
						}
					}


					for (int z = 0;z < Size_d;z++)
					{
						int loc = d[z];
						if (D[z] != source_ei)
						{
							graph[loc - 1][i] = true;
							graph_new[loc - 1][i] = abs(source_ei - DF[z]);

						}

					}


					for (int z = 0;z < Size_r2;z++)
					{
						int loc = r2[z];
						graph[loc - 1][i] = true;
						graph[i][loc - 1] = true;
						graph_new[loc - 1][i] = abs(source_ei - R[z]);
						graph_new[i][loc - 1] = abs(source_ei - R[z]);

					}

				}
			}


		}

	}


	int V = n, E = V * V;

	Graph g(V, E);


	for (int i = 0;i < n;i++)
	{
		for (int j = 0;j < n;j++)
			if (graph[i][j])
				g.addEdge(i, j, graph_new[i][j]);
	}


	int* RE = new int[m];
	for (int i = 0;i<m;i++)
		RE[i] = -1;
	int* indices = new int[m];

	int elevator_request = 0; // count number of elevators which are requests!
	for (int i = 0;i < m;i++)
	{

		bool flag = false;
		for (int j = 0;j < m;j++)
		{
			if (graph[l[j] - 1][l[i] - 1])
			{
				flag = true;
			}
		}

		bool flag2 = false;
		for (int j = 0;j<min.number;j++)
			if (l[i] == min.elements[j])
				flag2 = true;

		if (flag && !flag2) // elevator i is the request
		{
			RE[elevator_request] = l[i];
			indices[elevator_request] = i;
			elevator_request++;
		}
	}

	int z = ceil((Size_r2 + elevator_request) / min.number); // each elevator can server this number of floors
	int remaining = (Size_r2 + elevator_request) - z*min.number;

	cout << "z " << z << "remaining: " << remaining << " request " << elevator_request << endl;

	bool* visited_R = new bool[Size_r2 + elevator_request]; // size of r2
	for (int i = 0;i<Size_r2 + elevator_request;i++)
		visited_R[i] = false;


	for (int j = 0;j<min.number;j++)
	{
		bool flag = false;
		int index = -1;
		for (int i = 0;i<m;i++)
			if (l[i] == min.elements[j])
			{
				flag = true;
				index = i;
			}

		int E[3] = { 12,20,4 };

		int current = E[index];
		if (flag)
		{
			int size = z;
			if (remaining > 0)
			{
				size++;
				remaining--;
			}

			int* existing_r = new int[size];

			if (size > 0)
			{
				int count = 0;

				for (count = 0; count < size; count++)
				{
					int min = INT_MAX;
					int min_index = -1;
					for (int k = 0;k<Size_r2;k++)
						if (visited_R[k] == false)
						{
							if (abs(R[k] - current) < min)
							{
								min = abs(R[k] - current);
								min_index = k;

							}

						}
					for (int k = Size_r2;k<Size_r2 + elevator_request;k++)
						if (visited_R[k] == false)
						{
							if (abs(E[indices[k - Size_r2]] - current) < min)
							{
								min = abs(E[indices[k - Size_r2]] - current);
								min_index = k;
							}

						}
					visited_R[min_index] = true;
					if (min_index<Size_r2)
						existing_r[count] = r2[min_index];
					else
						existing_r[count] = RE[min_index - Size_r2];

				}

			}

			int* not_existing_r = new int[Size_r2 + elevator_request - size];
			int num = 0;
			for (int k = 0;k < Size_r2;k++)
			{
				bool flag_existing = false;
				for (int k1 = 0;k1<size;k1++)
					if (existing_r[k1] == r2[k])
						flag_existing = true;
				if (!flag_existing)
					not_existing_r[num++] = r2[k];
			}
			for (int k = 0;k < elevator_request; k++)
			{
				bool flag_existing = false;
				for (int k1 = 0;k1<size;k1++)
					if (existing_r[k1] == RE[k])
						flag_existing = true;
				if (!flag_existing)
					not_existing_r[num++] = RE[k];
			}

			bool flag_graph = false;
			for (int z = 0;z<min.number;z++)
				if (min.elements[z] == min.elements[j] && z != j)
					flag_graph = true;

			cout << endl;
		}

	}
}


void ElevatorStatistic(int* path, int size)
{

	int* cost = new int[size];
	cost[0] = 0;

	for (int i = 1;i < size;i++)
	{
		int first = path[i - 1];
		int second = path[i];

		/* for checking if it is an elevator and the source of that */
		int position_first = -1;
		for (int z = 0;z < Size_l;z++)
		{
			if (first == l[z])
			{
				position_first = E[z];
			}
		}

		/* for checking if it is a request and the source of that */
		for (int z = 0;z < Size_r2;z++)
		{
			if (first == r2[z])
			{
				position_first = R[z];
			}
		}

		/* for checking if it is a destination and the position of that */
		for (int z = 0;z < Size_d;z++)
		{
			if (first == d[z])
			{
				position_first = DF[z];
			}
		}


		/* for checking if it is an elevator and the source of that */
		int position_second = -1;
		for (int z = 0;z < Size_l;z++)
		{
			if (second == l[z])
			{
				position_second = E[z];
			}
		}

		/* for checking if it is a request and the source of that */
		for (int z = 0;z < Size_r2;z++)
		{
			if (second == r2[z])
			{
				position_second = R[z];
			}
		}

		/* for checking if it is a destination and the position of that */
		for (int z = 0;z < Size_d;z++)
		{
			if (second == d[z])
			{
				position_second = DF[z];
			}
		}

		cost[i] = cost[i - 1] + abs(position_first - position_second);

	}

	for (int i = 0;i < size;i++)
	{
		bool flag_d = false;
		int source_d = -1;
		int index = -1;

		for (int j = 0;j<Size_d;j++)
			if (d[j] == path[i])
			{
				index = j;

				flag_d = true;
				for (int m = 0;m<Size_l;m++)
					if (E[m] == D[j])
						source_d = l[m];
				for (int m = 0;m<Size_r2;m++)
					if (R[m] == D[j])
						source_d = r2[m];
			}
		int WaitingTime = 0;
		int TravelingTime = 0;
		if (flag_d)
			for (int k = 0;k<size;k++)
				if (path[k] == source_d)
				{
					WaitingTime = cost[k];
					TravelingTime = cost[i] - cost[k];

					d_visited[index] = 1;
					d_waitingtime[index] = WaitingTime;
					d_travelingtime[index] = TravelingTime;
					d_stops[index] = i - k;
				}
	}

	TotalCost += cost[size - 1];

	int Current_Users = 0;
	int Max_Users = 0;
	int destination_requests = 0;
	for (int i = 0;i < size;i++)
	{
		bool NotDestination = false;

		int first = path[i];

		/* for checking if it is an elevator and the source of that */
		int position_first = -1;
		for (int z = 0;z < Size_l;z++)
		{
			if (first == l[z])
			{
				position_first = E[z];

				NotDestination = true;
			}
		}

		/* for checking if it is a request and the source of that */
		for (int z = 0;z < Size_r2;z++)
		{
			if (first == r2[z])
			{
				position_first = R[z];

				NotDestination = true;

			}
		}

		/* for checking if it is a destination and the position of that */
		for (int z = 0;z < Size_d;z++)
		{
			if (first == d[z])
			{
				Current_Users--;
			}
		}

		if (NotDestination)
			for (int k = i + 1;k< size; k++)
				for (int j = 0;j<Size_d;j++)
					if (d[j] == path[k] && position_first == D[j])
					{
						Current_Users++;
						destination_requests++;
					}

		if (Current_Users>Max_Users)
			Max_Users = Current_Users;
	}

	cout << "Max users at a time at the elevator " << Max_Users << endl;
	cout << "Number of requests(destinations) served by this elevator  " << destination_requests << endl;
}

void statistics()
{

	int path1[12] = { 2,40,49,50,51,22,98,48,99,100,101,52 };
	ElevatorStatistic(path1, 12);

	int path2[11] = { 6,7,70,69,64,72,67,65,66,71,68 };
	ElevatorStatistic(path2, 11);

	int path3[11] = { 14,15,21,96,95,85,84,94,93,97,83 };
	ElevatorStatistic(path3, 11);

	int path4[11] = { 14,13,10,77,80,82,75,79,76,78,81 };
	ElevatorStatistic(path4, 11);

	int path5[12] = { 16,17,29,109,90,32,35,123,119,118,91,122 };
	ElevatorStatistic(path5, 12);

	int path6[12] = { 17,16,87,89,9,74,86,4,59,60,88,73 };
	ElevatorStatistic(path6, 12);

	int path7[10] = { 18,5,3,58,61,55,63,62,57,56 };
	ElevatorStatistic(path7, 10);

	int path8[12] = { 19,27,107,31,115,106,116,114,117,108,38,46 };
	ElevatorStatistic(path8, 12);

	int path9[12] = { 24,20,33,34,37,125,92,121,120,124,39,47 };
	ElevatorStatistic(path9, 12);

	int path10[12] = { 31,30,111,23,102,2,54,113,110,53,112,103 };
	ElevatorStatistic(path10, 12);

	int path11[10] = { 36,26,43,42,45,44,25,104,41,105 };
	ElevatorStatistic(path11, 10);

	cout << "Total cost of the elevators " << TotalCost << endl;

	int count = 0;
	for (int i = 0;i<Size_d;i++)
		if (d_visited[i] == 1)
			count++;

	cout << "Number of served destinations " << count << endl;

	int sum_waitingtime = 0;
	for (int i = 0;i<Size_d;i++)
		if (d_visited[i] == 1)
			sum_waitingtime += d_waitingtime[i];

	int sum_travelingtime = 0;
	for (int i = 0;i<Size_d;i++)
		if (d_visited[i] == 1)
			sum_travelingtime += d_travelingtime[i];

	int sum_experiencedstops = 0;
	for (int i = 0;i<Size_d;i++)
		if (d_visited[i] == 1)
			sum_experiencedstops += d_stops[i];

	double MeanWeight = (double)sum_waitingtime / (double)count;
	double MeanTravelingTime = (double)sum_travelingtime / (double)count;
	double MeanStops = (double)sum_experiencedstops / (double)count;


	cout << "mean waiting time " << MeanWeight << endl;
	cout << "mean traveling time " << MeanTravelingTime << endl;
	cout << "mean stops " << MeanStops << endl;

	double WaitingSquare = 0;
	for (int i = 0; i < Size_d; i++)
		if (d_visited[i] == 1)
			WaitingSquare += (d_waitingtime[i] - MeanWeight) *  (d_waitingtime[i] - MeanWeight);

	double WaitingVariance = (double)WaitingSquare / (double)(count);

	double TravelingSquare = 0;
	for (int i = 0; i < Size_d; i++)
		if (d_visited[i] == 1)
			TravelingSquare += (d_travelingtime[i] - MeanTravelingTime) *  (d_travelingtime[i] - MeanTravelingTime);

	double TravelingVariance = (double)TravelingSquare / (double)(count);

	int min_waiting = INT_MAX;
	int max_waiting = INT_MIN;

	for (int i = 0;i<Size_d;i++)
		if (d_visited[i] == 1)
		{
			if (d_waitingtime[i] < min_waiting)
				min_waiting = d_waitingtime[i];
			if (d_waitingtime[i] > max_waiting)
				max_waiting = d_waitingtime[i];
		}

	int min_traveling = INT_MAX;
	int max_traveling = INT_MIN;

	for (int i = 0;i<Size_d;i++)
		if (d_visited[i] == 1)
		{
			if (d_travelingtime[i] < min_traveling)
				min_traveling = d_travelingtime[i];
			if (d_travelingtime[i] > max_traveling)
				max_traveling = d_travelingtime[i];
		}

	cout << "Waiting Variance " << WaitingVariance << endl;
	cout << "Traveling Variance " << TravelingVariance << endl;
	cout << "Waiting Standard Deviation " << sqrt(WaitingVariance) << endl;
	cout << "Traveling Standard Deviation " << sqrt(TravelingVariance) << endl;
	cout << "Waiting Max " << max_waiting << endl;
	cout << "Waiting Min " << min_waiting << endl;
	cout << "Traveling Max " << max_traveling << endl;
	cout << "Traveling Min " << min_traveling << endl;
}

void GettingElevatorRequests()
{
	/* elevators which are requests as well in the first place */
	int** graph_new = new int*[n];
	for (int i = 0; i < n; ++i)
		graph_new[i] = new int[n];

	bool** graph = new bool*[n];
	for (int i = 0; i < n; ++i)
		graph[i] = new bool[n];

	for (int i = 0; i < n; ++i)
		for (int j = 0; j < n; ++j)
			graph[i][j] = false;

	for (int i = 0; i < n; ++i)
		for (int j = 0; j < n; ++j)
			graph_new[i][j] = C[i][j];


	for (int i = 0;i < n;i++)
	{

		/* for checking if it is an elevator and the source of that */
		int source_ei = -1;
		bool flag_ei = false;
		for (int z = 0;z < Size_l;z++)
		{
			if (i + 1 == l[z])
			{
				flag_ei = true;
				source_ei = E[z];
			}
		}


		/* for checking if it is a request and the source of that */
		int source_ri = -1;
		bool flag_ri = false;
		for (int z = 0;z < Size_r2;z++)
		{
			if (i + 1 == r2[z])
			{
				flag_ri = true;
				source_ri = R[z];
			}
		}

		/* for checking if it is a destination and the source of that */
		int source_di = -1;
		int dest_di = -1;
		bool flag_di = false;
		for (int z = 0;z < Size_d;z++)
		{
			if (i + 1 == d[z])
			{
				flag_di = true;
				source_di = D[z];
				dest_di = DF[z];
			}
		}
		/* end of checking */

		for (int j = 0;j < n;j++)
		{

			/* for checking if it is a request and the source of that */
			int source_rj = -1;
			bool flag_rj = false;
			for (int z = 0;z < Size_r2;z++)
			{
				if (j + 1 == r2[z])
				{
					flag_rj = true;
					source_rj = R[z];
				}
			}

			/* for checking if it is a destination and the source of that */
			int source_dj = -1;
			int dest_dj = -1;
			bool flag_dj = false;
			for (int z = 0;z < Size_d;z++)
			{
				if (j + 1 == d[z])
				{
					flag_dj = true;
					source_dj = D[z];
					dest_dj = DF[z];
				}
			}

			/* end of checking */

			if (flag_ei && flag_rj)
				graph[i][j] = true;
			if (flag_ri && flag_dj && source_ri == source_dj)
				graph[i][j] = true;
			if (flag_di && flag_dj)
			{
				if (i != j)
				{
					graph[i][j] = true;
					graph[j][i] = true;
					graph_new[i][j] = abs(dest_di - dest_dj);
					graph_new[j][i] = abs(dest_di - dest_dj);
				}
			}
			if (flag_di && flag_rj && source_rj != source_di)
				graph[i][j] = true;
			if (flag_ei && flag_dj && source_ei == source_dj)
			{
				graph[i][j] = true;
			}
			if (flag_ri && flag_rj)
			{
				if (i != j)
				{
					graph[i][j] = true;
					graph[j][i] = true;
				}
			}
			if (flag_ei && flag_dj && source_ei == source_dj)
			{
				graph[i][j] = true;
				graph_new[i][j] = abs(dest_dj - source_ei);

				for (int z = 0;z < Size_l;z++)
				{
					if (l[z] != i + 1)
					{

						int loc = l[z];

						graph[loc - 1][i] = true;
						graph_new[loc - 1][i] = abs(source_ei - E[z]);

					}
				}


				for (int z = 0;z < Size_d;z++)
				{
					int loc = d[z];
					if (D[z] != source_ei)
					{
						graph[loc - 1][i] = true;

						graph_new[loc - 1][i] = abs(source_ei - DF[z]);

					}

				}

				for (int z = 0;z < Size_r2;z++)
				{
					int loc = r2[z];
					graph[loc - 1][i] = true;
					graph[i][loc - 1] = true;
					graph_new[loc - 1][i] = abs(source_ei - R[z]);
					graph_new[i][loc - 1] = abs(source_ei - R[z]);

				}

			}

		}

	}



	int V = n, E = V * V;

	Graph g(V, E);

	for (int i = 0;i < n;i++)
	{
		for (int j = 0;j < n;j++)
			if (graph[i][j])
				g.addEdge(i, j, graph_new[i][j]);
	}


	/* build path and requests */

	int* RE = new int[m];
	for (int i = 0;i < m;i++)
		RE[i] = -1;
	int* indices = new int[m];

	int elevator_request = 0; // count number of elevators which are requests!
	for (int i = 0;i < m;i++)
	{

		bool flag = false;
		for (int j = 0;j < m;j++)
		{
			if (graph[l[j] - 1][l[i] - 1])
			{
				flag = true;
			}
		}


		if (flag) // elevator i is the request
		{
			RE[elevator_request] = l[i];

			indices[elevator_request] = i;
			cout << "number of elevator " << i << "  " << l[i] << endl;
			elevator_request++;
		}
	}
	system("pause");

}
void RCFM() {

	while (Round <= 13)
	{
		cout << "Round " << Round << endl;

		int** graph_new = new int*[n];
		for (int i = 0; i < n; ++i)
			graph_new[i] = new int[n];


		bool** graph = new bool*[n];
		for (int i = 0; i < n; ++i)
			graph[i] = new bool[n];

		for (int i = 0; i < n; ++i)
			for (int j = 0; j < n; ++j)
				graph[i][j] = false;


		for (int i = 0;i < n;i++)
		{

			/* for checking if it is an elevator and the source of that */
			int source_ei = -1;
			bool flag_ei = false;
			for (int z = 0;z < Size_l;z++)
			{
				if (i + 1 == l[z])
				{
					flag_ei = true;
					source_ei = E[z];
				}
			}


			/* for checking if it is a request and the source of that */
			int source_ri = -1;
			bool flag_ri = false;
			int Save_RequestRound_i = -1;
			for (int z = 0;z < Size_r2;z++)
			{
				if (i + 1 == r2[z])
				{
					flag_ri = true;
					source_ri = R[z];
					for (int counter = 0;counter < Size_r2 + Size_Elevator_Requests;counter++)
					{
						if (r2[z] == AllRequests[counter])
						{

							Save_RequestRound_i = PoissonArrival[counter];
						}
					}
				}
			}

			for (int z = 0;z < Size_Elevator_Requests;z++)
			{
				if (i + 1 == Elevator_Requests[z])
				{
					flag_ri = true;
					source_ri = Elevator_Requests_Sources[z];
					for (int counter = 0;counter < Size_r2 + Size_Elevator_Requests;counter++)
					{
						if (Elevator_Requests[z] == AllRequests[counter])
						{

							Save_RequestRound_i = PoissonArrival[counter];
						}
					}
				}
			}

			/* for checking if it is a destination and the source of that */
			int source_di = -1;
			int dest_di = -1;
			bool flag_di = false;
			int Save_DestinationRound_i = -1;
			for (int z = 0;z < Size_d;z++)
			{
				if (i + 1 == d[z])
				{
					flag_di = true;
					source_di = D[z];
					dest_di = DF[z];

					for (int counter = 0;counter < Size_r2 + Size_Elevator_Requests;counter++)
					{
						if (source_di == AllRequests_Sources[counter])
						{

							Save_DestinationRound_i = PoissonArrival[counter];
						}
					}
				}
			}
			/* end of checking */

			for (int j = 0;j < n;j++)
			{

				/* for checking if it is a request and the source of that */
				int source_rj = -1;
				bool flag_rj = false;
				int Save_RequestRound_j = -1;
				for (int z = 0;z < Size_r2;z++)
				{
					if (j + 1 == r2[z])
					{
						flag_rj = true;
						source_rj = R[z];

						for (int counter = 0;counter < Size_r2 + Size_Elevator_Requests;counter++)
						{
							if (r2[z] == AllRequests[counter])
							{

								Save_RequestRound_j = PoissonArrival[counter];
							}
						}
					}
				}

				/* for checking if it is an elevator-request and the source of that */

				for (int z = 0;z < Size_Elevator_Requests;z++)
				{
					if (j + 1 == Elevator_Requests[z])
					{
						flag_rj = true;
						source_rj = Elevator_Requests_Sources[z];

						for (int counter = 0;counter < Size_r2 + Size_Elevator_Requests;counter++)
						{
							if (Elevator_Requests[z] == AllRequests[counter])
							{

								Save_RequestRound_j = PoissonArrival[counter];
							}
						}
					}
				}

				/* for checking if it is a destination and the source of that */
				int source_dj = -1;
				int dest_dj = -1;
				bool flag_dj = false;
				int Save_DestinationRound_j = -1;
				for (int z = 0;z < Size_d;z++)
				{
					if (j + 1 == d[z])
					{
						flag_dj = true;
						source_dj = D[z];
						dest_dj = DF[z];

						for (int counter = 0;counter < Size_r2 + Size_Elevator_Requests;counter++)
						{
							if (source_dj == AllRequests_Sources[counter])
							{

								Save_DestinationRound_j = PoissonArrival[counter];
							}
						}
					}
				}

				/* end of checking */

				if (flag_ei && flag_rj && Save_RequestRound_j == Round)
				{
					graph[i][j] = true;
					if (source_ei >= source_rj)
						graph_new[i][j] = abs(source_ei - source_rj);
					else
						graph_new[i][j] = abs(source_ei - source_rj) + abs((source_ei - source_rj) / 10);
				}
				if (flag_ri && flag_dj && source_ri == source_dj && Save_RequestRound_i == Round && Save_DestinationRound_j == Round)
				{
					graph[i][j] = true;
					if (source_ri >= dest_dj)
						graph_new[i][j] = abs(source_ri - dest_dj);
					else
						graph_new[i][j] = abs(source_ri - dest_dj) + abs((source_ri - dest_dj) / 10);
				}
				if (flag_di && flag_dj && Save_DestinationRound_i == Round && Save_DestinationRound_j == Round)
				{
					if (i != j)
					{
						graph[i][j] = true;
						graph[j][i] = true;
						graph_new[i][j] = abs(dest_di - dest_dj);
						graph_new[j][i] = abs(dest_di - dest_dj);

						if (dest_di < dest_dj)
							graph_new[i][j] += abs(dest_di - dest_dj) / 10;

						if (dest_dj < dest_di)
							graph_new[j][i] += abs(dest_di - dest_dj) / 10;

					}
				}
				if (flag_di && flag_rj && source_rj != source_di && Save_DestinationRound_i == Round && Save_RequestRound_j == Round)
				{
					graph[i][j] = true;
					graph_new[i][j] = abs(dest_di - source_rj);

					if (dest_di < source_rj)
						graph_new[i][j] += abs(dest_di - source_rj) / 10;
				}
				if (flag_ri && flag_rj && Save_RequestRound_i == Round && Save_RequestRound_j == Round)
				{
					if (i != j)
					{
						graph[i][j] = true;
						graph_new[i][j] = abs(source_ri - source_rj);
						if (source_ri < source_rj)
							graph_new[i][j] += abs(source_ri - source_rj) / 10;

						graph[j][i] = true;
						graph_new[j][i] = abs(source_ri - source_rj);
						if (source_rj < source_ri)
							graph_new[j][i] += abs(source_ri - source_rj) / 10;
					}
				}


			}

		}



		int V = n, E = V * V;

		Graph g(V, E);
		int counter_edges = 0;
		for (int i = 0;i < n;i++)
		{
			for (int j = 0;j < n;j++)
				if (graph[i][j])
				{
					g.addEdge(i, j, graph_new[i][j]); counter_edges++;
				}
		}

		cout << "Counter number of edges in initial graph " << counter_edges << endl;
		system("pause");

		/* From here --- Here you must seperate the requests which are in Round and update Size_r2 */
		int Size_r2_New = 0;
		for (int count = 0; count < Size_r2 + Size_Elevator_Requests; count++)
		{
			if (AllRequests_Types[count] == 1 && PoissonArrival[count] == Round)
			{
				Size_r2_New++;
			}

		}
		cout << "just requests " << Size_r2_New << endl;

		int* r2_New = new int[Size_r2_New];
		int counter_r2_new = 0;
		for (int count = 0; count < Size_r2 + Size_Elevator_Requests; count++)
		{
			if (AllRequests_Types[count] == 1 && PoissonArrival[count] == Round)
			{
				r2_New[counter_r2_new++] = AllRequests[count];
				cout << "request " << AllRequests[count] << endl;
			}
		}

		/* From here --- Here you must seperate the requests which are initially are elevators as well and in Round */
		int Size_Initial_Elevator_Requests = 0;
		for (int count = 0; count < Size_r2 + Size_Elevator_Requests; count++)
		{
			if (AllRequests_Types[count] == 0 && PoissonArrival[count] == Round)
			{
				Size_Initial_Elevator_Requests++;
			}

		}
		cout << "initial elevator requests " << Size_Initial_Elevator_Requests << endl;

		int* Initial_Elevator_Requests = new int[Size_Initial_Elevator_Requests];
		int counter = 0;
		for (int count = 0; count < Size_r2 + Size_Elevator_Requests; count++)
		{
			if (AllRequests_Types[count] == 0 && PoissonArrival[count] == Round)
			{
				Initial_Elevator_Requests[counter++] = AllRequests[count];
			}

		}

		cout << "total requests selected in this round " << Size_r2_New + Size_Initial_Elevator_Requests << endl;

		int* Requests_Coalition_Index = new int[Size_r2_New + Size_Initial_Elevator_Requests];

		history* Requests_History = new history[Size_r2_New + Size_Initial_Elevator_Requests];

		for (int i = 0;i< Size_r2_New + Size_Initial_Elevator_Requests;i++)
			Requests_History[i].number = 0;

		for (int i = 0;i<Size_l;i++)
			Coalition[i].number = 0;
		int j;
		srand((unsigned int)time(NULL));
		bool flag;
		for (int i = 0;i < Size_r2_New;i++)
		{

			j = (rand() % Size_l);

			Coalition[j].elements[Coalition[j].number] = r2_New[i];
			Coalition[j].number = Coalition[j].number + 1;

			Requests_Coalition_Index[i] = j;

		}

		for (int i = 0;i < Size_Initial_Elevator_Requests;i++)
		{

			j = (rand() % Size_l);

			Coalition[j].elements[Coalition[j].number] = Initial_Elevator_Requests[i];
			Coalition[j].number = Coalition[j].number + 1;

			Requests_Coalition_Index[i + Size_r2_New] = j;

		}
		for (int i = 0;i < Size_l;i++)
		{

			int* existing_r = new int[Coalition[i].number];
			for (int m = 0;m<Coalition[i].number;m++)
				existing_r[m] = Coalition[i].elements[m];


			bool flag_graph = false;
			Edges* SelectedEdges = BuildGraph(l[i], i, graph, graph_new, existing_r, Coalition[i].number, flag_graph);

			if (Num_SelectedEdges <= p)
			{
				int cost = 0;
				for (int i = 0;i < Num_SelectedEdges;i++)
				{
					cost += SelectedEdges[i].weight;
				}
				Coalition[i].value = cost;

			}
			else
			{

				Coalition[i].value = INT_MAX;
			}

			Num_SelectedEdges = 0;
		}

		int counter_switch = 0;
		bool check_mark = true;
		while (check_mark)
		{

			check_mark = false;
			for (int i = 0;i < Size_r2_New;i++)
			{
				int current = Requests_Coalition_Index[i];
				int best = Requests_Coalition_Index[i];

				double Best_Cost = (double)(Coalition[best].value) / (double)(Coalition[best].number);


				for (int j = 0;j < Size_l;j++)
				{
					bool flag = true;

					for (int k = 0;k < Requests_History[i].number;k++)
					{
						if (Requests_History[i].elevators[k] == j)
							flag = false;
					}
					if (j != current && Coalition[j].value != INT_MAX && flag) 
					{
						int* existing_r = new int[Coalition[j].number + 1];
						for (int m = 0;m<Coalition[j].number;m++)
							existing_r[m] = Coalition[j].elements[m];
						existing_r[Coalition[j].number] = r2_New[i];

						bool flag_graph = false;
						Edges* SelectedEdges = BuildGraph(l[j], i, graph, graph_new, existing_r, Coalition[j].number + 1, flag_graph);

						if (Num_SelectedEdges <= p)
						{
							int cost = 0;
							for (int i = 0;i<Num_SelectedEdges;i++)
								cost += SelectedEdges[i].weight;

							double New_Cost = (double)(cost) / (double)(Coalition[j].number + 1);

							if (New_Cost < Best_Cost)
							{
								Best_Cost = New_Cost;
								best = j;
							}
						}

						Num_SelectedEdges = 0;

					}
				}

				if (best != current)
				{

					/* first step */
					int number = Requests_History[i].number;
					Requests_History[i].elevators[number] = current;
					Requests_History[i].number++;

					/* second step */
					int counter = 0;
					int* existing_r = new int[Coalition[current].number - 1];

					for (int m = 0;m<Coalition[current].number;m++)
						if (Coalition[current].elements[m] != r2_New[i])
							existing_r[counter++] = Coalition[current].elements[m];


					Coalition[current].number--;

					for (int m = 0;m<Coalition[current].number;m++)
						Coalition[current].elements[m] = existing_r[m];


					bool flag_graph = false;
					Edges* SelectedEdges = BuildGraph(l[current], current, graph, graph_new, existing_r, Coalition[current].number, flag_graph);

					if (Num_SelectedEdges <= p)
					{
						int cost = 0;
						for (int i = 0;i<Num_SelectedEdges;i++)
							cost += SelectedEdges[i].weight;
						Coalition[current].value = cost;

					}

					Num_SelectedEdges = 0;

					/* third step */

					Coalition[best].elements[Coalition[best].number] = r2_New[i];
					Coalition[best].number = Coalition[best].number + 1;

					Requests_Coalition_Index[i] = best;

					int* existing_r2 = new int[Coalition[best].number];

					for (int m = 0;m<Coalition[best].number;m++)
						existing_r2[m] = Coalition[best].elements[m];

					flag_graph = false;
					Edges* SelectedEdges2 = BuildGraph(l[best], best, graph, graph_new, existing_r2, Coalition[best].number, flag_graph);

					if (Num_SelectedEdges <= p)
					{
						int cost = 0;
						for (int i = 0;i<Num_SelectedEdges;i++)
							cost += SelectedEdges2[i].weight;

						Coalition[best].value = cost;

					}

					Num_SelectedEdges = 0;

					check_mark = true;

					counter_switch++;
				}
			}

			/* end the first part */

			for (int i = 0;i < Size_Initial_Elevator_Requests;i++)
			{
				int current = Requests_Coalition_Index[i + Size_r2_New];
				int best = Requests_Coalition_Index[i + Size_r2_New];

				double Best_Cost = (double)(Coalition[best].value) / (double)(Coalition[best].number);


				for (int j = 0;j < Size_l;j++)
				{
					bool flag = true;

					for (int k = 0;k < Requests_History[i + Size_r2_New].number;k++)
					{
						if (Requests_History[i + Size_r2_New].elevators[k] == j)
							flag = false;
					}
					if (j != current && Coalition[j].value != INT_MAX && flag)
					{
						int* existing_r = new int[Coalition[j].number + 1];
						for (int m = 0;m<Coalition[j].number;m++)
							existing_r[m] = Coalition[j].elements[m];
						existing_r[Coalition[j].number] = Initial_Elevator_Requests[i];


						bool flag_graph = false;
						Edges* SelectedEdges = BuildGraph(l[j], j, graph, graph_new, existing_r, Coalition[j].number + 1, flag_graph);

						if (Num_SelectedEdges <= p)
						{
							int cost = 0;
							for (int i = 0;i<Num_SelectedEdges;i++)
								cost += SelectedEdges[i].weight;

							double New_Cost = (double)(cost) / (double)(Coalition[j].number + 1);

							if (New_Cost < Best_Cost)
							{
								Best_Cost = New_Cost;
								best = j;
							}
						}

						Num_SelectedEdges = 0;

					}
				}

				if (best != current)
				{

					int number = Requests_History[i + Size_r2_New].number;
					Requests_History[i + Size_r2_New].elevators[number] = current;
					Requests_History[i + Size_r2_New].number++;


					int counter = 0;
					int* existing_r = new int[Coalition[current].number - 1];

					for (int m = 0;m<Coalition[current].number;m++)
						if (Coalition[current].elements[m] != Initial_Elevator_Requests[i])
							existing_r[counter++] = Coalition[current].elements[m];


					Coalition[current].number--;

					for (int m = 0;m<Coalition[current].number;m++)
						Coalition[current].elements[m] = existing_r[m];


					bool flag_graph = false;
					Edges* SelectedEdges = BuildGraph(l[current], current, graph, graph_new, existing_r, Coalition[current].number, flag_graph);

					if (Num_SelectedEdges <= p)
					{
						int cost = 0;
						for (int i = 0;i<Num_SelectedEdges;i++)
							cost += SelectedEdges[i].weight;

						Coalition[current].value = cost;
					}

					Num_SelectedEdges = 0;


					Coalition[best].elements[Coalition[best].number] = Initial_Elevator_Requests[i];
					Coalition[best].number = Coalition[best].number + 1;

					Requests_Coalition_Index[i + Size_r2_New] = best;

					int* existing_r2 = new int[Coalition[best].number];

					for (int m = 0;m<Coalition[best].number;m++)
						existing_r2[m] = Coalition[best].elements[m];


					flag_graph = false;
					Edges* SelectedEdges2 = BuildGraph(l[best], best, graph, graph_new, existing_r2, Coalition[best].number, flag_graph);

					if (Num_SelectedEdges <= p)
					{
						int cost = 0;
						for (int i = 0;i<Num_SelectedEdges;i++)
							cost += SelectedEdges2[i].weight;

						Coalition[best].value = cost;

					}

					Num_SelectedEdges = 0;


					check_mark = true;

					counter_switch++;
				}
			}

		}
		cout << "Result Ready!\n";

		int cost = 0;

		flag_print = true;
		flag_round = true;

		for (int i = 0;i < Size_l;i++)
		{

			if (Coalition[i].number != 0)
			{
				cout << "number " << Coalition[i].number << " Elevator " << i + 1 << endl;
				for (int m = 0; m < Coalition[i].number; m++)
					cout << "member " << Coalition[i].elements[m] << endl;
			}
			int* existing_r = new int[Coalition[i].number];
			for (int m = 0;m<Coalition[i].number;m++)
				existing_r[m] = Coalition[i].elements[m];

			bool flag_graph = false;
			if (Coalition[i].number != 0)
			{
				Edges* SelectedEdges = BuildGraph(l[i], i, graph, graph_new, existing_r, /*types,*/ Coalition[i].number, flag_graph);

				cout << "elevator: " << i + 1 << "  " << Num_SelectedEdges << "  " << Coalition[i].value << endl;

				for (int j = 0;j<Num_SelectedEdges;j++)
					cost += SelectedEdges[j].weight;

				cout << " elevator " << " cost " << cost << endl;
			}


			Num_SelectedEdges = 0;

		}

		Round++;
		flag_print = false;
		flag_round = false;
		cout << "counter_switch " << counter_switch << endl;
		cout << "cost " << cost << endl;
		system("pause");

	}
}

void IPTest(bool* VisitedNodes)
{
	for (int i = 0; i < n; i++)
		if (VisitedNodes[i] == 1)
			cout << " node in the graph " << i + 1 << endl;

	system("pause");
}
void IP_GraphInitialization()
{
	while (Round <= 13)
	{
		int total_unvisited = 0;
		cout << "Round " << Round << endl;

		int** graph_new = new int*[n];
		for (int i = 0; i < n; ++i)
			graph_new[i] = new int[n];


		bool** graph = new bool*[n];
		for (int i = 0; i < n; ++i)
			graph[i] = new bool[n];

		for (int i = 0; i < n; ++i)
			for (int j = 0; j < n; ++j)
				graph[i][j] = false;


		for (int i = 0;i < n;i++)
		{

			/* for checking if it is an elevator and the source of that */
			int source_ei = -1;
			bool flag_ei = false;
			for (int z = 0;z < Size_l;z++)
			{
				if (i + 1 == l[z])
				{
					flag_ei = true;
					source_ei = E[z];
				}
			}


			/* for checking if it is a request and the source of that */
			int source_ri = -1;
			bool flag_ri = false;
			int Save_RequestRound_i = -1;
			for (int z = 0;z < Size_r2;z++)
			{
				if (i + 1 == r2[z])
				{
					flag_ri = true;
					source_ri = R[z];
					for (int counter = 0;counter < Size_r2 + Size_Elevator_Requests;counter++)
					{
						if (r2[z] == AllRequests[counter])
						{

							Save_RequestRound_i = PoissonArrival[counter];
						}
					}
				}
			}

			/* for checking if it is an elevator-request and the source of that */

			for (int z = 0;z < Size_Elevator_Requests;z++)
			{
				if (i + 1 == Elevator_Requests[z])
				{
					flag_ri = true;
					source_ri = Elevator_Requests_Sources[z];
					for (int counter = 0;counter < Size_r2 + Size_Elevator_Requests;counter++)
					{
						if (Elevator_Requests[z] == AllRequests[counter])
						{

							Save_RequestRound_i = PoissonArrival[counter];
						}
					}
				}
			}

			/* for checking if it is a destination and the source of that */
			int source_di = -1;
			int dest_di = -1;
			bool flag_di = false;
			int Save_DestinationRound_i = -1;
			for (int z = 0;z < Size_d;z++)
			{
				if (i + 1 == d[z])
				{
					flag_di = true;
					source_di = D[z];
					dest_di = DF[z];

					for (int counter = 0;counter < Size_r2 + Size_Elevator_Requests;counter++)
					{
						if (source_di == AllRequests_Sources[counter])
						{

							Save_DestinationRound_i = PoissonArrival[counter];
						}
					}
				}
			}
			/* end of checking */

			for (int j = 0;j < n;j++)
			{
				/* for checking if it is an elevator and the source of that */
				int source_ej = -1;
				bool flag_ej = false;
				for (int z = 0;z < Size_l;z++)
				{
					if (j + 1 == l[z])
					{
						flag_ej = true;
						source_ej = E[z];
					}
				}

				/* for checking if it is a request and the source of that */
				int source_rj = -1;
				bool flag_rj = false;
				int Save_RequestRound_j = -1;
				for (int z = 0;z < Size_r2;z++)
				{
					if (j + 1 == r2[z])
					{
						flag_rj = true;
						source_rj = R[z];

						for (int counter = 0;counter < Size_r2 + Size_Elevator_Requests;counter++)
						{
							if (r2[z] == AllRequests[counter])
							{

								Save_RequestRound_j = PoissonArrival[counter];
							}
						}
					}
				}

				/* for checking if it is an elevator-request and the source of that */
				for (int z = 0;z < Size_Elevator_Requests;z++)
				{
					if (j + 1 == Elevator_Requests[z])
					{
						flag_rj = true;
						source_rj = Elevator_Requests_Sources[z];

						for (int counter = 0;counter < Size_r2 + Size_Elevator_Requests;counter++)
						{
							if (Elevator_Requests[z] == AllRequests[counter])
							{

								Save_RequestRound_j = PoissonArrival[counter];
							}
						}
					}
				}

				/* for checking if it is a destination and the source of that */
				int source_dj = -1;
				int dest_dj = -1;
				bool flag_dj = false;
				int Save_DestinationRound_j = -1;
				for (int z = 0;z < Size_d;z++)
				{
					if (j + 1 == d[z])
					{
						flag_dj = true;
						source_dj = D[z];
						dest_dj = DF[z];

						for (int counter = 0;counter < Size_r2 + Size_Elevator_Requests;counter++)
						{
							if (source_dj == AllRequests_Sources[counter])
							{

								Save_DestinationRound_j = PoissonArrival[counter];
							}
						}
					}
				}

				/* end of checking */

				if ((flag_ei || (flag_ri && Save_RequestRound_i == Round) || (flag_di && Save_DestinationRound_i == Round)) && (flag_ej || (flag_rj && Save_RequestRound_j == Round) || (flag_dj && Save_DestinationRound_j == Round)) && (i == j))
				{
					graph[i][j] = true;
					graph_new[i][j] = maxint;
				}
				else if (flag_ri && flag_ej && Save_RequestRound_i == Round)
				{
					graph[i][j] = true;
					graph_new[i][j] = 0;
				}
				else if (flag_di && flag_ej && Save_DestinationRound_i == Round)
				{
					graph[i][j] = true;
					graph_new[i][j] = 0;
				}
				else if (flag_ei && flag_ej)
				{
					graph[i][j] = true;

					graph_new[i][j] = maxint;
				}
				else if (flag_ei && flag_dj && Save_DestinationRound_j == Round)
				{
					graph[i][j] = true;

					graph_new[i][j] = maxint;
				}

				else if (flag_di && flag_dj && Save_DestinationRound_i == Round && Save_DestinationRound_j == Round )
				{
					if (i != j)
					{
						graph[i][j] = true;
						graph[j][i] = true;
						graph_new[i][j] = abs(dest_di - dest_dj);
						graph_new[j][i] = abs(dest_di - dest_dj);

						if (dest_di < dest_dj)
							graph_new[i][j] += abs(dest_di - dest_dj) / 10;

						if (dest_dj < dest_di)
							graph_new[j][i] += abs(dest_di - dest_dj) / 10;

					}
				}
				else if (flag_ri && flag_dj && source_ri == source_dj && Save_RequestRound_i == Round && Save_DestinationRound_j == Round)
				{
					graph[i][j] = true;
					if (source_ri >= dest_dj)
						graph_new[i][j] = abs(source_ri - dest_dj);
					else
						graph_new[i][j] = abs(source_ri - dest_dj) + abs((source_ri - dest_dj) / 10);

					graph[j][i] = true;
					graph_new[j][i] = maxint;
				}
				else if (flag_di && flag_rj && source_rj != source_di && Save_DestinationRound_i == Round && Save_RequestRound_j == Round)
				{
					graph[i][j] = true;
					graph_new[i][j] = abs(dest_di - source_rj);

					if (dest_di < source_rj)
						graph_new[i][j] += abs(dest_di - source_rj) / 10;
				}

				else if (flag_ri && flag_dj && source_ri != source_dj && Save_RequestRound_i == Round && Save_DestinationRound_j == Round)
				{
					graph[i][j] = true;
					graph_new[i][j] = maxint;
				}
				else if (flag_di && flag_dj && source_di != source_dj && Save_DestinationRound_i == Round && Save_DestinationRound_j == Round)
				{
					graph[i][j] = true;
					graph_new[i][j] = maxint;
				}
				else if (flag_ei && flag_rj && Save_RequestRound_j == Round)
				{
					graph[i][j] = true;
					if (source_ei >= source_rj)
						graph_new[i][j] = abs(source_ei - source_rj);
					else
						graph_new[i][j] = abs(source_ei - source_rj) + abs((source_ei - source_rj) / 10);
				}

				else if (flag_ri && flag_rj && Save_RequestRound_i == Round && Save_RequestRound_j == Round)
				{
					if (i != j)
					{
						graph[i][j] = true;
						graph_new[i][j] = abs(source_ri - source_rj);
						if (source_ri < source_rj)
							graph_new[i][j] += abs(source_ri - source_rj) / 10;

						graph[j][i] = true;
						graph_new[j][i] = abs(source_ri - source_rj);
						if (source_rj < source_ri)
							graph_new[j][i] += abs(source_ri - source_rj) / 10;
					}
				}
				if ((flag_ei || (flag_ri && Save_RequestRound_i == Round) || (flag_di && Save_DestinationRound_i == Round)) && (flag_ej || (flag_rj && Save_RequestRound_j == Round) || (flag_dj && Save_DestinationRound_j == Round)) && !graph[i][j])
				{
					total_unvisited++;
				}

			}

		}

		cout << "unvisited " << total_unvisited << endl;
		system("pause");


		bool* visitednodes = new bool[n];
		for (int count = 0; count < n; count++)
			visitednodes[count] = 0;

		int V = n, E = V * V;
		Graph g(V, E);

		int counter_edges = 0;
		int count_unvisited = 0;

		for (int i = 0;i < n;i++)
		{
			for (int j = 0;j < n;j++)
				if (graph[i][j])
				{
					visitednodes[i] = 1;
					visitednodes[j] = 1;
					g.addEdge(i, j, graph_new[i][j]);
					counter_edges++;
				}
		}
		for (int i = 0;i < n;i++)
		{
			for (int j = 0;j < n;j++)
				if (graph[i][j])
				{
				}
				else if (visitednodes[i] && visitednodes[j] && !graph[i][j])
					count_unvisited++;
		}

		int count_nodes = 0;
		for (int i = 0;i < n;i++)
			if (visitednodes[i] == 1)
				count_nodes++;

		int* MainIndices = new int[count_nodes];
		int start = 0;
		for (int i = 0;i<n;i++)
			if (visitednodes[i] == 1)
			{
				MainIndices[start++] = i + 1;
			}

		cout << "The number of edges " << counter_edges << "The number of nodes " << count_nodes << " count unvisited " << count_unvisited << endl;

		cout << " start " << start << endl;
		for (int i = 0;i < start;i++)
			cout << MainIndices[i] << endl;
		system("pause");

		int** graph_round = new int*[count_nodes];
		for (int i = 0; i < count_nodes; ++i)
			graph_round[i] = new int[count_nodes];

		cout << "hello world" << count_nodes << endl;
		int count_edges = 0;
		for (int i = 0;i < n;i++)
		{
			for (int j = 0;j < n;j++)
				if (graph[i][j])
				{
					count_edges++;
				}
		}
		cout << "edges " << count_edges << endl;
		system("pause");
		int counter_graph_round_i = 0;
		int counter_graph_round_j = 0;
		int count_final = 0;
		for (int i = 0; i < n; i++)
			for (int j = 0; j < n; j++)
				if (graph[i][j])
				{
					if (counter_graph_round_j <= count_nodes - 1 && counter_graph_round_i <= count_nodes - 1)
					{
						graph_round[counter_graph_round_i][counter_graph_round_j] = graph_new[i][j];
						counter_graph_round_j++;
						count_final++;
					}
					else if (counter_graph_round_j == count_nodes  && counter_graph_round_i < count_nodes - 1)
					{
						counter_graph_round_j = 0;
						counter_graph_round_i++;
						graph_round[counter_graph_round_i][counter_graph_round_j] = graph_new[i][j];
						counter_graph_round_j++;
						count_final++;
					}
				}

		
		coalition temp;
		temp.number = 20;

		temp.elements[0] = l[0];
		temp.elements[1] = l[1];
		temp.elements[2] = l[2];
		temp.elements[3] = l[3];
		temp.elements[4] = l[4];
		temp.elements[5] = l[5];
		temp.elements[6] = l[6];
		temp.elements[7] = l[7];
		temp.elements[8] = l[8];
		temp.elements[9] = l[9];
		temp.elements[10] = l[10];
		temp.elements[11] = l[11];
		temp.elements[12] = l[12];
		temp.elements[13] = l[13];
		temp.elements[14] = l[14];
		temp.elements[15] = l[15];
		temp.elements[16] = l[16];
		temp.elements[17] = l[17];
		temp.elements[18] = l[18];
		temp.elements[19] = l[19];

		temp.value = 0;
		temp.reviewed = false;
		cout << "results " << temp.value << endl;

		temp.value = IP(temp, visitednodes, MainIndices, graph_round, count_nodes);

		cout << "results " << temp.value << endl;
		system("pause");
		Round++;
	}
}
void InputTransformation()
{
	int round = 1;

	int Source_Floors[125] = { 10,12,13,14,15,17,18,19,21,22,23,24,25,27,28,29,2,30,31,33,34,35,37,38,39,3,40,41,42,43,44,45,47,48,49,4,50,5,7,8,3,3,3,3,3,5,7,8,8,8,8,8,12,12,13,13,13,13,14,14,15,15,15,17,17,17,17,17,18,18,18,18,21,21,22,22,22,22,25,25,25,25,28,28,28,29,29,29,29,30,30,33,34,34,34,34,34,35,35,35,35,37,37,39,39,40,40,40,42,43,43,43,43,44,44,44,44,45,45,47,48,49,49,50,50 };
	int Floor_types[125] = { 1,1,2,2,2,1,2,1,2,1,1,1,2,1,2,1,1,1,1,2,2,2,2,1,2,2,2,1,2,1,1,2,2,1,2,1,2,2,2,1,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4 };
	int Destination_floors[125] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,48,12,7,31,22,25,14,42,10,15,25,4,22,18,40,50,47,18,19,22,27,46,42,6,3,34,4,49,7,20,37,4,49,20,11,34,15,37,9,15,45,14,45,12,21,18,27,39,25,45,9,17,37,11,27,31,40,48,21,15,7,15,9,31,50,46,43,14,42,20,41,11,19,25,43,46,19,10,31,13,17,5,48,11,18 };
	
	int n = 125;
	cout << "Elevators" << endl;
	for (int i = 0;i < Size_l;i++)
	{
		int index = -1;
		for (int k = 0;k < n;k++)
			if (Floor_types[k] == 1 && Source_Floors[k] == E[i])
				index = k;
		cout << E[i] << "e" << "   index: " << index + 1 << "  " << endl;
	}

	while (round <= 13)
	{
		cout << "Round " << round << endl;
		for (int i = 0; i < Size_r2 + Size_Elevator_Requests; i++)
		{
			if (PoissonArrival[i] == round)
			{
				if (AllRequests_Types[i] == 1)
				{
					int index = -1;
					for (int k = 0;k < n;k++)
						if (Floor_types[k] == 2 && Source_Floors[k] == AllRequests_Sources[i])
							index = k;
					cout << AllRequests_Sources[i] << "r" << "   index: " << index + 1 << "  " << endl;

					for (int j = 0;j < Size_d;j++)
					{
						if (D[j] == AllRequests_Sources[i])
						{
							int index = -1;
							for (int k = 0;k < n;k++)
								if (Floor_types[k] == 4 && Source_Floors[k] == D[j] && Destination_floors[k] == DF[j])
									index = k;
							cout << D[j] << "d" << DF[j] << "   index: " << index + 1 << "  " << endl;
						}

					}
				}
				else if (AllRequests_Types[i] == 0)
				{
					int index = -1;
					for (int k = 0;k < n;k++)
						if (Floor_types[k] == 1 && Source_Floors[k] == AllRequests_Sources[i])
							index = k;
					cout << AllRequests_Sources[i] << "er" << "   index: " << index + 1 << "  " << endl;

					for (int j = 0;j < Size_d;j++)
					{
						if (D[j] == AllRequests_Sources[i])
						{
							int index = -1;
							for (int k = 0;k < n;k++)
								if (Floor_types[k] == 4 && Source_Floors[k] == D[j] && Destination_floors[k] == DF[j])
									index = k;
							cout << D[j] << "d" << DF[j] << "   index: " << index + 1 << "  " << endl;
						}

					}
				}
			}

		}
		round++;
	}

}
int main(int, char**) {

	InputTransformation();
	system("pause");

	for (int i = 0; i < n; i++)
	{
		bool flag_ei = false;
		int source_ei = -1;
		for (int j = 0; j < Size_l; j++)
		{
			if (i + 1 == l[j])
			{
				flag_ei = true;
				source_ei = E[j];
			}
		}


		bool flag_ri = false;
		int source_ri = -1;

		for (int j = 0; j < Size_r2; j++)
		{
			if (i + 1 == r2[j])
			{
				flag_ri = true;
				source_ri = R[j];
			}
		}
		bool flag_eri = false;
		int source_eri = -1;

		for (int j = 0; j < Size_Elevator_Requests; j++)
		{
			if (i + 1 == Elevator_Requests[j])
			{
				flag_eri = true;
				source_eri = Elevator_Requests_Sources[j];
			}
		}

		bool flag_di = false;
		int source_di = -1;
		int dest_di = -1;
		for (int j = 0; j < Size_d; j++)
		{
			if (i + 1 == d[j])
			{
				flag_di = true;
				source_di = D[j];
				dest_di = DF[j];
			}
		}

	}


	auto start = high_resolution_clock::now();

	RCFM();

	auto stop = high_resolution_clock::now();

	auto duration = duration_cast<microseconds>(stop - start);

	cout << "Time taken by function: "
		<< duration.count() << " microseconds" << endl;
	system("pause");
	return 0;
}

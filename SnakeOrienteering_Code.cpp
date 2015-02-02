#include <iostream>
#include <vector>
#include <map>
#include <set>
#include<cmath>
#include <cstdlib>
#include <cstdio>
using namespace std;

//Position of a character in the map.
struct Position
{
	int x;
	int y;
};

//This will represent the attributes/information about that particular position in the map.
//This includes the heuristic value of that position and the actual distance from the start point.
struct Attributes
{
	Position Ps;
	int Ds;
	int Hs;
};

//This is the main Base Class, containing all the required declarations of all the public and private functions.
class DynamicTspTraversal
{
    public:
        void main();
        int Tsp_Traversal();
        int InitialiseVariables();
        void Initialise_Nearby_Elements(const Attributes At);
        Attributes Nearby_Position[4];


    private:

        void Create_DistanceMatrix();

        int Astar_Search(const Position& start, const Position& end);
        int Find_MinimumDistance(int n, int cur);
        bool Could_Reach();
        Position Start, Goal;
        int Width;
        int Height;
        int LeastCostPath;
        bool Is_Visited[18];
        vector<vector<char> > OrienteeringMap;
        map<long, int> DistanceMatrix;
        vector<Position> Check_Points;
        map<long, int> Memoised_Distance;



};

//Definitions for the greaterthan(<),lessthan(>) and equalsto(==) operators that can be applied on a given position in the map.


bool operator==(const Position &a, const Position &b) {
	return a.x == b.x && a.y == b.y;
}

bool operator<(const Position &a, const Position &b) {
	return a.x * 100 + a.y < b.x * 100 + b.y;
}

bool operator<(const Attributes &a, const Attributes &b) {
	return a.Ps < b.Ps;
}


//This is the heuristic function. Here we have taken the absolute distance between two points as their heuristic value.

double Calc_Heuristic(const Position& start, const Position& end){
	return sqrt((start.x - end.x)*(start.x - end.x) + (start.y - end.y)*(start.y - end.y));
}

//This function represent the actual position of the coordinate in the DistanceMatrix.

long Checkpoint_Position(const Position& start, const Position& end){
	if (start < end) {
		return Checkpoint_Position(end, start);
	}
	return (start.x * 100 + start.y) * 10000 + end.x * 100 + end.y;
}

//This function takes the input and collects the checkpoints in the vector Check_Points for future reference.
int DynamicTspTraversal::InitialiseVariables()
 {
	cin >> Width >> Height;
	if (Width > 100 || Height > 100) {
		return -1;
	}
	for (int i = 0; i < Height; i++)
    {
		vector<char> tmp;
		for (int j = 0; j < Width; j++)
		{
			char c;
			cin >> c;
			tmp.push_back(c);
			Position x = {j, i};
			switch(c)
			{
			case 'S':
				Start = x;
				break;
			case '.':
			case '#':
				break;
			case '@':
				Check_Points.push_back(x);
				break;
            case 'G':
				Goal = x;
				break;
			default:
				return -1;
			}
		}
		OrienteeringMap.push_back(tmp);
	}
	if (Check_Points.size() > 18) {
		return -1;
	}
	return 0;
}
void DynamicTspTraversal::Initialise_Nearby_Elements(const Attributes At){


		//for motion in +x/-x direction
		Nearby_Position[0].Ps.x = At.Ps.x + 1;
		Nearby_Position[2].Ps.x = At.Ps.x - 1;
		Nearby_Position[0].Ps.y = At.Ps.y;
		Nearby_Position[2].Ps.y = At.Ps.y;

		//for motion in +y/-y direction
		Nearby_Position[1].Ps.y = At.Ps.y + 1;
		Nearby_Position[3].Ps.y = At.Ps.y - 1;
		Nearby_Position[1].Ps.x = At.Ps.x;
		Nearby_Position[3].Ps.x = At.Ps.x;

}

//As the name implies this function has the standard Astar search implementation that can be used in case of graph represented in the form of a matrix.
//It requires just the starting vertex and the final ending vertex for the search.
int DynamicTspTraversal::Astar_Search(const Position& start, const Position& end)
{
	map<long,int>::iterator it;
	it = DistanceMatrix.find(Checkpoint_Position(start, end));
	if (it != DistanceMatrix.end()) {
		return it->second;
	}

	set<Attributes> Min_Set;
	set<Attributes> Temp_Set;
	//Also tried vectors here but using set was most helpful because of internal predefined 'find' method
	//which is not available in vectors.

	Attributes s;
	s.Ps = start;
	s.Ds = 0;
	s.Hs = Calc_Heuristic(start, end);
	Temp_Set.insert(s);

	while (!Temp_Set.empty())
    {
		set<Attributes>::iterator MinC;
		int NewMin = 1000;
		for (set<Attributes>::iterator i = Temp_Set.begin(); i != Temp_Set.end(); i++){
			if (i->Ds + i->Hs < NewMin){
                MinC = i;
				NewMin = i->Ds + i->Hs;
			}
		}
		Attributes At = *MinC;
		Temp_Set.erase(MinC);
		Min_Set.insert(At);

		if (At.Ps == end)
        {   set<Attributes>::iterator i;
			for (i = Min_Set.begin(); i != Min_Set.end(); i++) {
				int x = i->Ps.x;
				int y = i->Ps.y;
				if (OrienteeringMap[y][x] == '@' || OrienteeringMap[y][x] == 'G') {
					DistanceMatrix[Checkpoint_Position(start, i->Ps)] = i->Ds;
				}
			}
			return At.Ds + At.Hs;
		}

		Initialise_Nearby_Elements(At);

		for (int i = 0; i < 4; i++)
        {
			int x = Nearby_Position[i].Ps.x;
			int y = Nearby_Position[i].Ps.y;

			if (OrienteeringMap[y][x] != '#' && Min_Set.find(Nearby_Position[i]) == Min_Set.end())
			{
				Nearby_Position[i].Ds = At.Ds + 1;
				Nearby_Position[i].Hs = Calc_Heuristic(Nearby_Position[i].Ps, end);

				set<Attributes>::iterator it = Temp_Set.find(Nearby_Position[i]);
				if (it != Temp_Set.end()){
					if(Nearby_Position[i].Ds < it->Ds) {
						Temp_Set.erase(it);
						Temp_Set.insert(Nearby_Position[i]);
					}
				}
				else {
					Temp_Set.insert(Nearby_Position[i]);
				}
			}
		}
	}
	return -1;
}

//This function checks whether all the coordinates could be reached from the starting vertex or not.
bool DynamicTspTraversal::Could_Reach(){

	if (Astar_Search(Start, Goal) == -1)
		return false;
	vector<Position>::iterator i;
	for (i = Check_Points.begin(); i != Check_Points.end(); i++) {
		if (Astar_Search(Start, *i) == -1)
			return false;
	}
	return true;
}

//This function creates the DistanceMatrix that contains the distances between all the vertices of the graph.
void DynamicTspTraversal::Create_DistanceMatrix() {
    //This loop calculates the shortest path between starting vertex
    // and the checkpoints via Astar Algo. and saves it to the DistanceMatrix.
	for (int i = 0; i < Check_Points.size(); i++) {
		if (DistanceMatrix.find(Checkpoint_Position(Start, Check_Points[i])) == DistanceMatrix.end()) {
			Astar_Search(Start, Check_Points[i]);
		}
	}

	 //This loop calculates the shortest paths between the checkpoints via Astar Algo. and saves it to the DistanceMatrix.
	for (int i = 0; i < Check_Points.size(); i++) {
		for (int j = 1; j < Check_Points.size(); j++) {
			if (DistanceMatrix.find(Checkpoint_Position(Check_Points[i], Check_Points[j])) == DistanceMatrix.end()) {
				Astar_Search(Check_Points[i], Check_Points[j]);
			}
		}
	}

	 //This loop calculates the shortest path between Goal vertex
    // and the checkpoints via Astar Algo. and saves it to the DistanceMatrix.
	for (int i = 0; i < Check_Points.size(); i++) {
		if (DistanceMatrix.find(Checkpoint_Position(Goal, Check_Points[i])) == DistanceMatrix.end()) {
			Astar_Search(Goal, Check_Points[i]);
		}
	}

}

//This function finds the minimum distance path between a pair via memoised recursive traversal through all the paths.
int DynamicTspTraversal::Find_MinimumDistance(int num, int cur) {

	if (num == 0) {
		return DistanceMatrix[Checkpoint_Position(Goal, Check_Points[cur])];
	}

	long tmp = 0;
	//Using the bool Is_Visited array and the previous tmp value  to create the final value of tmp where
	//we can store the memoised distance for easy reference.
	for(int i = 0; i < Check_Points.size(); i++) {
		tmp = (tmp << 1) + Is_Visited[i];
	}
	//Multiplying tmp with 100 as the curr value ranges from 0 to 18.
	tmp = tmp * 100 + cur;
	//If this path was calculated earlier then we directly return the memoised value instead of recalculating the path.
    map<long,int>::iterator it;
    it = Memoised_Distance.find(tmp);
	if (it != Memoised_Distance.end()) {
		return it->second;
	}

	int LeastCostPath = 10000;
	for(int i = 0; i < Check_Points.size(); i++) {
		if(!Is_Visited[i]) {
			int temp=0;
			Is_Visited[i] = 1;
			//Recursive LeastCostPath calculation ....
			temp=DistanceMatrix[Checkpoint_Position(Check_Points[i], Check_Points[cur])]+Find_MinimumDistance(num-1, i);
			Is_Visited[i] = 0;

			if(temp < LeastCostPath) {
				LeastCostPath = temp;
			}

		}
	}
    //Storing the calculated LeastCostPath value in Memoised_Distance vector for future references
    //to eliminate the recalculation of that path.
	Memoised_Distance[tmp] = LeastCostPath;
	return LeastCostPath;
}

int DynamicTspTraversal::Tsp_Traversal() {

	if (!Could_Reach())
		return -1;

    //If the checkpoint's vector is empty then return the Astar search from start to goal as we
    //not need to traverse through any checkpoints now.
	if (Check_Points.empty())
		{return Astar_Search(Start, Goal);}

	Create_DistanceMatrix();

	for(int i = 0; i < Check_Points.size(); i++) {
		Is_Visited[i] = 0;
	}

	LeastCostPath = 10000;
	//Here this loop calculates the minimum distance path by considering each and every checkpoint in the probable traversed minimum path.
	for(int i = 0; i < Check_Points.size(); i++) {
        int temp=0;
        Is_Visited[i] = 1;
        temp=DistanceMatrix[Checkpoint_Position(Check_Points[i], Start)]+Find_MinimumDistance(Check_Points.size() - 1, i);
        Is_Visited[i] = 0;

		if (temp < LeastCostPath)
			{LeastCostPath = temp;}
	}
	return LeastCostPath;
}


int main(int argc, char* argv[])
{
        DynamicTspTraversal o;
		o.InitialiseVariables();
        cout<<o.Tsp_Traversal();
        return 0;
}




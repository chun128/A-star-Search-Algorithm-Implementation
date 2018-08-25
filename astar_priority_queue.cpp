#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cfloat>
#include <cstring>
#include <vector>
#include <iomanip>
#include <list>
#include <cmath>
#include <queue>


using namespace std;

class Grid
{
public:
    // constructor
    Grid()
    {
        _sourceCost = DBL_MAX;
        _totalCost = DBL_MAX;
        _pi = NULL;
        _isPath = false;
        _isDone = false;
    }

    // public member functions
    int getX() const { return _x; }
    int getY() const { return _y; }
    bool isPath() const { return _isPath; }
    bool isDone() const { return _isDone; }
    Grid *getPi() const { return _pi; }
    double getSourceCost() const { return _sourceCost; }
    double getTotalCost() const { return _totalCost; }

    void setX(int x) { _x = x; }
    void setY(int y) { _y = y; }
    void setPath() { _isPath = true; }
    void setDone() { _isDone = true; }
    void setPi(Grid *pi) { _pi = pi; }
    void setSourceCost(double sourceCost) { _sourceCost = sourceCost; }
    // To Do:
    // You might need to add some member function to compute the _totalCost 
    // whenever the _sourceCost is updated.
	void setTotalCost(int x, int y, int tarX, int tarY) { 
		double h = abs(x-tarX) + abs(y-tarY);
		_totalCost = _sourceCost + h;}
	
private:
    // data members
    int _x, _y; // x & y coordinates
    bool _isPath;
    bool _isDone;
    double _sourceCost, _totalCost;
    Grid *_pi;
};
struct cmp
{
	bool operator()(const Grid *lhs, const Grid *rhs) 
	{
		return lhs->getTotalCost() > rhs->getTotalCost();
	}
};

// global variable
vector< vector<Grid> > map;
vector< vector<double> > horizontalEdge;
vector< vector<double> > verticalEdge;
priority_queue<Grid *, vector<Grid *>, cmp > priorityQ;
int boundaryX, boundaryY;
int sourceX, sourceY;
int targetX, targetY;

// functions
void relax(Grid *u, Grid *v, double w);
Grid *extractMin();
void printMap();
void printEdgeWeights();

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        cout << "Usage: ./astar [input_case]" << endl;
        system("pause");
        exit(EXIT_FAILURE);
    }

    /* parse the input file */
    fstream inFile;
    inFile.open(argv[1], fstream::in);
    if(!inFile.is_open())
    {
        cout << "The input file is not opened!" << endl;
        system("pause");
        exit(EXIT_FAILURE);
    }

    char buffer[100];

    while(inFile >> buffer)
    {
        if(strcmp(buffer, "boundary") == 0)
        {
            inFile >> buffer;
            boundaryX = atoi(buffer);
            inFile >> buffer;
            boundaryY = atoi(buffer);
        }
        else if(strcmp(buffer, "source") == 0)
        {
            inFile >> buffer;
            sourceX = atoi(buffer);
            inFile >> buffer;
            sourceY = atoi(buffer);
        }
        else if(strcmp(buffer, "target") == 0)
        {
            inFile >> buffer;
            targetX = atoi(buffer);
            inFile >> buffer;
            targetY = atoi(buffer);
        }
        else if(strcmp(buffer, "horizontal") == 0)
        {
            inFile >> buffer >> buffer;
            horizontalEdge.resize(boundaryX);
            for(int i = 0; i < boundaryX; ++i)
            {
                horizontalEdge[i].resize(boundaryY - 1);
            }
            for(int i = 0; i < boundaryX; ++i)
            {
                for(int j = 0; j < boundaryY - 1; ++j)
                {
                    inFile >> buffer;
                    horizontalEdge[i][j] = atof(buffer);
                }
            }
        }
        else if(strcmp(buffer, "vertical") == 0)
        {
            inFile >> buffer >> buffer;
            verticalEdge.resize(boundaryX - 1);
            for(int i = 0; i < boundaryX - 1; ++i)
            {
                verticalEdge[i].resize(boundaryY);
            }
            for(int i = 0; i < boundaryX - 1; ++i)
            {
                for(int j = 0; j < boundaryY; ++j)
                {
                    inFile >> buffer;
                    verticalEdge[i][j] = atof(buffer);
                }
            }
        }
        else
        {
            cout << "Unmatched string in the input file!" << endl;
        }
    }

    /* initialize the routing map */
    map.resize(boundaryX);
    for(int i = 0; i < boundaryX; ++i)
    {
        map[i].resize(boundaryY);
    }

    for(int i = 0; i < boundaryX; ++i)
    {
        for(int j = 0; j < boundaryY; ++j)
        {
            map[i][j].setX(i);
            map[i][j].setY(j);
            if((i == sourceX && j == sourceY) || ( i == targetX && j == targetY))
                map[i][j].setPath();
        }
    }

    cout << "--------------------" << endl;
    cout << "   Initial Map" << endl;
    cout << "--------------------" << endl;
    printMap();
    printEdgeWeights();
    
    priorityQ.push(&map[sourceX][sourceY]);
    map[sourceX][sourceY].setSourceCost(0);
    //map[sourceX][sourceY].setTotalCost(0,abs(targetX-sourceX+targetY-sourceY));
    
    /* A-Star */
    // To Do:
    while(!priorityQ.empty() && map[targetX][targetY].isDone()==false)
	{
		  Grid *u, *v;
		  double w;
		  u=extractMin();
		  //if(u->getX() == targetX && u->getY() == targetY)break;
		  map[u->getX()][u->getY()].setDone();
		  	  
		  if((u->getX()+1) < boundaryX ) //&& (targetX-(u->getX()+1) + targetY-(u->getY())) < (targetX-(u->getX()) + targetY-(u->getY()))
		  {
		  	v=&map[u->getX()+1][u->getY()];
		  	w=verticalEdge[u->getX()][u->getY()];
		  	relax(u, v, w);
		  	v->setTotalCost(u->getX()+1, u->getY(), targetX, targetY);
			if(v->isDone()==false)
			{
				priorityQ.push(v);
				v->setDone();
			}
		  }
		  if(u->getX() > 0 )
		  {
		  	v=&map[u->getX()-1][u->getY()];
		  	w=verticalEdge[u->getX()-1][u->getY()];
		  	relax(u, v, w);
		  	v->setTotalCost(u->getX()-1, u->getY(), targetX, targetY);
			if(v->isDone()==false)
			{
				priorityQ.push(v);
				v->setDone();
			}
		  }
		  if((u->getY()+1) < boundaryY )
		  {
		  	v=&map[u->getX()][u->getY()+1];
		  	w=horizontalEdge[u->getX()][u->getY()];
		  	relax(u, v, w);
		  	v->setTotalCost(u->getX(), u->getY()+1, targetX, targetY);
			if(v->isDone()==false)
			{
				priorityQ.push(v);
				v->setDone();
			}
		  }
		  if(u->getY() > 0 )
		  {
		  	v=&map[u->getX()][u->getY()-1];
		  	w=horizontalEdge[u->getX()][u->getY()-1];
		  	relax(u, v, w);
		  	v->setTotalCost(u->getX(), u->getY()-1, targetX, targetY);
			if(v->isDone()==false)
			{
				priorityQ.push(v);
				v->setDone();
			}
		  }
	}
    
    Grid *t, *m;
    double min;
    t=&map[targetX][targetY];
	while(t!=&map[sourceX][sourceY])
    {
		m=t->getPi(); //©¹¦^§ä 
		m->setPath();
		t=m;                                                                                     
    } 
    
    cout << " " << endl;
	cout << "--------------------" << endl;
    cout << "   Final Map" << endl;
    cout << "--------------------" << endl;
    
	printMap();
	cout << "-------------------------------------------" << endl;
	cout << "Total cost from the source to the target : " << map[targetX][targetY].getTotalCost() << endl;
    
    system("pause");
    return 0;
}

void relax(Grid *u, Grid *v, double w)
{
    // To Do:
    if(v->getSourceCost() > u->getSourceCost()+w)
    {
    	v->setSourceCost(u->getSourceCost()+w);
    	v->setPi(u);
	}
}

Grid *extractMin()
{
	Grid *minGrid = priorityQ.top();
    priorityQ.pop();
    return minGrid;
}

void printMap()
{
    cout << "Routing map:" << endl;
    for(int i = 0; i < map.size(); ++i)
    {
        for(int j = 0; j < map[0].size(); ++j)
        {
            if(map[i][j].isPath())
                cout << setw(2) << "*";
            else
                cout << setw(2) << "-";
        }
        cout << endl;
    }

    cout << "pi map:" << endl;
    for(int i = 0; i < map.size(); ++i)
    {
        for(int j = 0; j < map[0].size(); ++j)
        {
            if(map[i][j].getPi() == NULL)
                cout << "- ";
            else
            {
                Grid *current = &map[i][j];
                Grid *pi = map[i][j].getPi();
                if(pi->getX() < current->getX())
                    cout << "u ";
                else if(pi->getX() > current->getX())
                    cout << "d ";
                else if(pi->getY() < current->getY())
                    cout << "l ";
                else
                    cout << "r ";
            }
        }
        cout << endl;
    }
}

void printEdgeWeights()
{
    cout << "Horizontal edge weigths:" << endl;
    for(int i = 0; i < horizontalEdge.size(); ++i)
    {
        for(int j = 0; j < horizontalEdge[0].size(); ++j)
        {
            cout << setprecision(2) << fixed << horizontalEdge[i][j] << " ";
        }
        cout << endl;
    }
    cout << "Vertical edge weigths:" << endl;
    for(int i = 0; i < verticalEdge.size(); ++i)
    {
        for(int j = 0; j < verticalEdge[0].size(); ++j)
        {
            cout << setprecision(2) << fixed << verticalEdge[i][j] << " ";
        }
        cout << endl;
    }
}


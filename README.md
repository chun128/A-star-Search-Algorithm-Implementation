# A*-Search-Algorithm-Implementation
Implement the A* algorithm to find the shortest distance from the source to the target<br>
Use the “parent” information to reconstruct the shortest path from the source to the target

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

Develop on Dev C++

[Here is the link for Dev C++ downloading](https://sourceforge.net/projects/orwelldevcpp/)


## Running the tests

![image](https://github.com/chun128/Dijkstra-sAlgorithm-Implementation/blob/master/readme%20picture/test%20case.JPG)

Given:<br/>
* The size of a grid map<br/>
* The coordinates of a source and a target<br/>
* Horizontal/vertical edge weights<br/>

Parse the input file: <br>
```
Usage: ./astar [input_case]
```

![image](https://github.com/chun128/A-star-Search-Algorithm-Implementation/blob/master/readme%20pictures/initial%20map.jpg)

### Priority Queue
use std::priority_queue <br>
Operator overloading on priority_queue of object pointers<br>
```
priority_queue<Grid *, vector<Grid *>, cmp > priorityQ;
```

Operator overloading "cmp"
```
struct cmp
{
	bool operator()(const Grid *lhs, const Grid *rhs) 
	{
		return lhs->getTotalCost() > rhs->getTotalCost();
	}
};
```


## Execution Result

Show the routing map and pi map as below

![image](https://github.com/chun128/A-star-Search-Algorithm-Implementation/blob/master/readme%20pictures/final%20map.jpg)


## Built With

* [Dev C++](http://www.bloodshed.net/devcpp.html) - The compiler used
 

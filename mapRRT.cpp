// Original RRT algo repo:
//https://github.com/nikhilchandak/Rapidly-Exploring-Random-Trees 

// To run the RRT and run the waypoint and colreg mission directly after the waypoints have been generated:
//./sfml-app ; cd; cd moos-ivp-tschaefe/missions/m2_berta_blackout ; ktm; ./clean.sh; ./launch.sh

// To test square.PNG or im_with_coord.png:
//modify start stop coordinates
//modify image name and height width
//modify threshold type
//modify jump size
//modify biased sampling zone


//includes files and namespaces ---------------------------------------

#include <chrono>
#include <cmath>
#include <iostream>
#include <cstdio>
#include <random>
#include <SFML/Graphics.hpp>
#include <geometry.h>
#include <fstream>
namespace MyLib 
{
    using GeometryPoint = Point;
}
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
using namespace cv;
using namespace std ; 


//visualization and simulation parameters--------------------------------------------------------------------------------


bool see_image_processing_only = false;
bool visualize_sampling_area = false;
bool visualize_sampling = false; 
int nb_max_iterations = 1000 ;
int nb_max_iterations_after_found = 1000;
bool optimize_sampling_nodes = true; 
bool optimize_sampling_obstacles = false; 

int nb_simulations = 0;
int nb_total_simulations = 1;


//image to process------------------------------------------------------------------------------------------------------

//string image_name = "im_with_coord.png";
//const int WIDTH = 1683 ;
//const int HEIGHT = 925 ;

//string image_name = "MIT_SP_boats.png";
//const int WIDTH = 3989 ;
//const int HEIGHT = 2333 ;

//string image_name = "square.PNG";
//const int WIDTH = 164 ;
//const int HEIGHT = 129 ;

//string image_name = "smartRRTtest.png";
//const int WIDTH = 240 ;
//const int HEIGHT = 229 ;

//string image_name = "narrowpass.png";
//const int WIDTH = 600 ;
//const int HEIGHT = 453 ;

//string image_name = "bugtrap.png";
//const int WIDTH = 600 ;
//const int HEIGHT = 453 ;

string image_name = "complex_env.png";
const int WIDTH = 601 ;
const int HEIGHT = 451 ;

//string image_name = "M2.png";
//const int WIDTH = 416 ;
//const int HEIGHT = 413 ;

//string image_name = "M3.png";
//const int WIDTH = 413 ;
//const int HEIGHT = 412 ;

//string image_name = "M4.png";
//const int WIDTH = 411 ;
//const int HEIGHT = 409 ;

//string image_name = "M5.png";
//const int WIDTH = 450 ;
//const int HEIGHT = 450 ;

//string image_name = "M6.png";
//const int WIDTH = 442 ;
//const int HEIGHT = 443 ;



//RRT parameters and variables (declared as global variables, very bad practice) -----------------------------------------

const int RADIUS = 5 ; 
const double GOAL_SAMPLING_PROB = 0.05;
const double INF = 1e18;

//const double JUMP_SIZE = (WIDTH/100.0 * HEIGHT/100.0)/1.5; //for im_with_coord.png
//const double JUMP_SIZE = (WIDTH/100.0 * HEIGHT/100.0)/10; //for MIT_SP.png
//const double JUMP_SIZE = 5; //for square.PNG
//const double JUMP_SIZE = 55; //for narrowpass.png
//const double JUMP_SIZE = 55; //for M3.png
const double JUMP_SIZE = (WIDTH+HEIGHT)/9 ; //tentative for all maps (except bugtrap)
//const double JUMP_SIZE = 75; //for bugtrap.png
//const double JUMP_SIZE = 15; //for smartRRT.png
const double DISK_SIZE = JUMP_SIZE ; // Ball radius around which nearby points are found 

int path_point_id =0;
int obstacle_id =0;
int obstacle_vertex_id =0;

int iterations = 0 ; int iterations_after_found =0; //added here, was declared in main before, did it to use it in RRT() modulo 3 to biase the sampling around obstacle edge

vector < Polygon > obstacles ; 
MyLib::GeometryPoint start, stop ; 
int obstacle_cnt = 1 ;

vector<MyLib::GeometryPoint> nodes ; 
vector<MyLib::GeometryPoint> path_points ; 

vector<int> parent, nearby ; 
vector < double > cost, jumps ; 
int nodeCnt = 0, goalIndex = -1 ; 

vector <sf::ConvexShape> polygons ;
sf::CircleShape startingPoint, endingPoint ; 
bool pathFound = 0 ;

template <typename T> // Returns a random number in [low, high] 
T randomCoordinate(T low, T high){
    random_device random_device;
    mt19937 engine{random_device()};
    uniform_real_distribution<double> dist(low, high);
    return dist(engine);
}

MyLib::GeometryPoint pickRandomPointAround(double x, double y, double distance) {
    double random_sample = randomCoordinate(0.0, 1.0); 
    if((random_sample - GOAL_SAMPLING_PROB) <= EPS and !pathFound) return stop + MyLib::GeometryPoint(RADIUS, RADIUS) ;
	return MyLib::GeometryPoint(randomCoordinate( x - distance, x + distance), randomCoordinate( y - distance, y + distance)); 
}


////////////////////////Rendering reesizable window

void resizeView(const sf::RenderWindow& window, sf::View& view) {
    float windowRatio = float(window.getSize().x) / float(window.getSize().y);
    float viewRatio = float(WIDTH) / float(HEIGHT);
    float sizeX = 1;
    float sizeY = 1;
    float posX = 0;
    float posY = 0;

    if (windowRatio > viewRatio) {
        // If window's width is bigger than view's width (compared with their respective heights)
        // We need to adjust the size of x
        sizeX = windowRatio / viewRatio;
        posX = -(WIDTH * (sizeX - 1)) / 2.f;
    } else {
        // If window's height is bigger than view's height (compared with their respective widths)
        // We need to adjust the size of y
        sizeY = viewRatio / windowRatio;
        posY = -(HEIGHT * (sizeY - 1)) / 2.f;
    }

    view.setSize(WIDTH * sizeX, HEIGHT * sizeY);
    view.setViewport(sf::FloatRect(posX / WIDTH, posY / HEIGHT, sizeX, sizeY));
}



MyLib::GeometryPoint convert_to_pmarineviewer(MyLib::GeometryPoint point_pixel) 
{
//values found in .tif file
//double lat_north = 43.826105; 
//double lat_south = 43.822142;
//double lon_east  = -70.326919;
//double lon_west  = -70.332412;

//values found in MIT_SP.tif file
double lat_north = 42.360264; 
double lat_south = 42.35415;
double lon_east  = -71.080058;
double lon_west  = -71.094214;

//values calculated with internet calculator, distances to the origin point in the .tif file: https://www.movable-type.co.uk/scripts/latlong.html
double x_west = -555.9;
double x_east = 607.2;
double y_north = 203.3;
double y_south = -476.6;

//WIDTH = 1157;
//HEIGHT = 856 ;
double x_meter = (point_pixel.x - 0)/(WIDTH - 0) * (x_east-x_west) + x_west;
double y_meter = (point_pixel.y - 0)/(HEIGHT-0)  * ( y_south-y_north ) + y_north;

return MyLib::GeometryPoint( x_meter, y_meter);
		
}

MyLib::GeometryPoint convert_latlon_to_pixels(double lat, double lon) 
{

//values found in MIT_SP.tif file
double lat_north = 42.360264; 
double lat_south = 42.35415;
double lon_east  = -71.080058;
double lon_west  = -71.094214;

double x_pixel;
double y_pixel;

//values calculated with internet calculator, distances to the origin point in the .tif file: https://www.movable-type.co.uk/scripts/latlong.html
if ((lat >= lat_south) and (lat<=lat_north) and (lon >= lon_west) and (lon <= lon_east))
{
	 x_pixel = WIDTH*( (lon-lon_west)/(lon_east-lon_west));
	 y_pixel = HEIGHT*( (lat_north-lat)/(lat_north-lat_south));
}

else {
 x_pixel = 0;
 y_pixel = 0;
}
return MyLib::GeometryPoint( x_pixel, y_pixel);
		
}


// Prepares SFML objects of starting, ending point and obstacles 
void prepareInput() {
	// Make starting and ending point circles ready 
	startingPoint.setRadius(RADIUS); endingPoint.setRadius(RADIUS); 
    startingPoint.setFillColor(sf::Color(208, 0, 240)); endingPoint.setFillColor(sf::Color::Blue);
    startingPoint.setPosition(start.x, start.y); endingPoint.setPosition(stop.x, stop.y);
    startingPoint.setOrigin(RADIUS/2, RADIUS/2); endingPoint.setOrigin(RADIUS/2, RADIUS/2);

    // Prepare polygon of obstacles 
	polygons.resize(obstacle_cnt);
	for(int i = 0; i < obstacle_cnt; i++) {
		polygons[i].setPointCount(obstacles[i].pointCnt); 
		polygons[i].setFillColor(sf::Color(89, 87, 98)); 
		for(int j = 0; j < obstacles[i].pointCnt; j++) 
			polygons[i].setPoint(j, sf::Vector2f(obstacles[i].points[j].x, obstacles[i].points[j].y));
	}
}

void draw(sf::RenderWindow& window) {
	sf::Vertex line[2]; sf::CircleShape nodeCircle;

	// Creating our shape.
	  sf::RectangleShape rectangle(sf::Vector2f(10.0f,20.0f));
	  rectangle.setFillColor(sf::Color::Red);
	  rectangle.setPosition(320,240);
	  rectangle.setOrigin(rectangle.getSize().x / 2, rectangle.getSize().y / 2);
	 // window.draw(rectangle); 

	// Draw edges between nodes 
	for(int i = (int)nodes.size() - 1; i; i--) {
		MyLib::GeometryPoint par = nodes[parent[i]] ; 
		line[0] = sf::Vertex(sf::Vector2f(par.x, par.y));
		line[1] = sf::Vertex(sf::Vector2f(nodes[i].x, nodes[i].y));
		line[0].color  = sf::Color::Green;
		line[1].color  = sf::Color::Green;
		window.draw(line, 2, sf::Lines);
	}

	window.draw(startingPoint); window.draw(endingPoint);

	// If destination is reached then path is retraced and drawn 
	if(pathFound) {
		int node = goalIndex; 
		path_points.clear();
		
		while(parent[node] != node) {
			int par = parent[node];
			line[0] = sf::Vertex(sf::Vector2f(nodes[par].x, nodes[par].y));
			line[1] = sf::Vertex(sf::Vector2f(nodes[node].x, nodes[node].y));
			line[0].color = line[1].color = sf::Color::Red; // orange color 
			
			//Create rectangles instead of lines for the main path, in order to be able to set the thickness of the line and make it more visible
			    sf::Vector2f p1(nodes[par].x, nodes[par].y);
			    sf::Vector2f p2(nodes[node].x, nodes[node].y);
			    
			    // Calculate the length and angle of the "line"
			    float length = sqrt((p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y));
			    float angle = atan2(p2.y - p1.y, p2.x - p1.x) * 180 / M_PI;

			    sf::RectangleShape line_rectangle(sf::Vector2f(length, 6)); // 2 is the thickness
			    line_rectangle.setPosition(p1);
			    line_rectangle.setRotation(angle);
			    line_rectangle.setFillColor(sf::Color::Red); // Set color to red
			
			
			path_points.push_back(nodes[node]);
			
			window.draw(line_rectangle);
			//window.draw(line, 2, sf::Lines);
			
			if (visualize_sampling_area)
			{
			//sf::RectangleShape rectangle(sf::Vector2f(1.5*distance(nodes[par], nodes[node]),1.5*distance(nodes[par], nodes[node]))); // adaptative area
			sf::RectangleShape rectangle(sf::Vector2f(200,200)); //fixed area
			  rectangle.setFillColor(sf::Color::Red);
			  rectangle.setOutlineColor(sf::Color::Red);
			  rectangle.setPosition(nodes[par].x,nodes[par].y);
			  rectangle.setOrigin(rectangle.getSize().x / 2, rectangle.getSize().y / 2);
			  window.draw(rectangle);  
			 }
			node = par ;
		}
		
		if (visualize_sampling)
		{
		for (int l = 0; l <path_points.size(); l++ )
		
		{
		
		MyLib::GeometryPoint newRandomPoint = pickRandomPointAround(path_points[l].x,path_points[l].y, 100);
		
		
		
		sf::RectangleShape rectangle(sf::Vector2f(20,20));
			  rectangle.setFillColor(sf::Color::Blue);
			  rectangle.setOutlineColor(sf::Color::Blue);
			  rectangle.setPosition(newRandomPoint.x,newRandomPoint.y);
			  rectangle.setOrigin(rectangle.getSize().x / 2, rectangle.getSize().y / 2);
			  window.draw(rectangle); 
		
		} 
		}
		
		
		
		
	}
}



// Returns true if the line segment ab is obstacle free
bool isEdgeObstacleFree(MyLib::GeometryPoint a, MyLib::GeometryPoint b) {
    for(auto& poly: obstacles)
        if(lineSegmentIntersectsPolygon(a, b, poly))
        	return false ; 
    return true ; 
}

// Returns a random point with some bias towards goal 
MyLib::GeometryPoint pickRandomPoint() {
    double random_sample = randomCoordinate(0.0, 1.0); 
    if((random_sample - GOAL_SAMPLING_PROB) <= EPS and !pathFound) return stop + MyLib::GeometryPoint(RADIUS, RADIUS) ;
	return MyLib::GeometryPoint(randomCoordinate(0, WIDTH), randomCoordinate(0, HEIGHT)); 
}


void checkDestinationReached() {
	sf::Vector2f position = endingPoint.getPosition(); 
	if(checkCollision(nodes[parent[nodeCnt - 1]], nodes.back(), MyLib::GeometryPoint(position.x, position.y), RADIUS)) {
		pathFound = 1 ; 
		goalIndex = nodeCnt - 1;
		cout << "Reached!! With a distance of " << cost.back() << " units. " << endl << endl ; 
	}
}

/* Inserts nodes on the path from rootIndex till Point q such 
   that successive nodes on the path are not more than 
   JUMP_SIZE distance away */
void insertNodesInPath(int rootIndex, MyLib::GeometryPoint& q) {
	MyLib::GeometryPoint p = nodes[rootIndex] ; 
	if(!isEdgeObstacleFree(p, q)) return ;
	while(!(p == q)) {
		MyLib::GeometryPoint nxt = p.steer(q, JUMP_SIZE); 
		nodes.push_back(nxt); 
		parent.push_back(rootIndex);
		cost.push_back(cost[rootIndex] + distance(p, nxt));
		rootIndex = nodeCnt++ ; 
		p = nxt ; 
	}
}

/*  Rewires the parents of the tree greedily starting from 
	the new node found in this iterationsation as the parent */
void rewire() {
	int lastInserted = nodeCnt - 1 ; 
	for(auto nodeIndex: nearby) {
		int par = lastInserted, cur = nodeIndex;

		// Rewire parents as much as possible (greedily)
		while( ((cost[par] + distance(nodes[par], nodes[cur])) - cost[cur]) <= EPS) { 
		
			if(!isEdgeObstacleFree(nodes[par], nodes[cur])) return ; //try to add this line to not rewire if not obstacle free
			int oldParent = parent[cur] ;
			parent[cur] = par; cost[cur] = cost[par] + distance(nodes[par], nodes[cur]);
			par = cur, cur = oldParent; 
		}
	}
}

/*	Runs one iteration of RRT depending on user choice 
	At least one new node is added on the screen each iteration. */
void RRT() {
	MyLib::GeometryPoint newPoint, nearestPoint, nextPoint ; bool updated = false ; int cnt = 0 ; 
	int nearestIndex = 0 ; double minCost = INF; nearby.clear(); jumps.resize(nodeCnt); 

	while(!updated) {
		
		if(!pathFound)
		{
		newPoint = pickRandomPoint(); 
		// Find nearest point to the newPoint such that the next node 
		// be added in graph in the (nearestPoint, newPoint) while being obstacle free
		nearestPoint = *nodes.begin(); nearestIndex = 0;
		for(int i = 0; i < nodeCnt; i++) {
			if(pathFound and randomCoordinate(0.0, 1.0) < 0.25) // Recalculate cost once in a while 
				cost[i] = cost[parent[i]] + distance(nodes[parent[i]], nodes[i]);  

			// Make smaller jumps sometimes to facilitate passing through narrow passages 
			jumps[i] = randomCoordinate(0.3, 1.0) * JUMP_SIZE ; 
			//jumps[i] = JUMP_SIZE ;
			auto pnt = nodes[i] ; 
			if((pnt.distance(newPoint) - nearestPoint.distance(newPoint)) <= EPS and isEdgeObstacleFree(pnt, pnt.steer(newPoint, jumps[i])))
				nearestPoint = pnt, nearestIndex = i ; 
		}
		nextPoint = stepNear(nearestPoint, newPoint, jumps[nearestIndex]);
		if(!isEdgeObstacleFree(nearestPoint, nextPoint)) continue ; 


		// Find nearby nodes to the new node as center in ball of radius DISK_SIZE 
		for(int i = 0; i < nodeCnt; i++)
			if((nodes[i].distance(nextPoint) - DISK_SIZE) <= EPS and isEdgeObstacleFree(nodes[i], nextPoint))
				nearby.push_back(i);

		// Find minimum cost path to the new node 
		int par = nearestIndex; minCost = cost[par] + distance(nodes[par], nextPoint);
		for(auto nodeIndex: nearby) {
			if( ( (cost[nodeIndex] + distance(nodes[nodeIndex], nextPoint)) - minCost) <= EPS)
				minCost = cost[nodeIndex] + distance(nodes[nodeIndex], nextPoint), par = nodeIndex;
		}

		parent.push_back(par); cost.push_back(minCost);
		nodes.push_back(nextPoint); nodeCnt++; 
		updated = true ; 
		if(!pathFound) checkDestinationReached(); 
		rewire();
		
		}
		if(pathFound)
		{
		
			
			//cout << "Found";
			
    		
    		
    		//optimize sampling area based on current path nodes
		
		
		
		if ((path_points.size() !=0) and optimize_sampling_nodes )
		{
		
		
		if (path_point_id > path_points.size() )
		{
		path_point_id = 0;
		}
		
		//newPoint = pickRandomPointAround(path_points[path_point_id].x,path_points[path_point_id].y, 50); //for im_with_coord.png
		//newPoint = pickRandomPointAround(path_points[path_point_id].x,path_points[path_point_id].y, 5); //for square.png
		newPoint = pickRandomPointAround(path_points[path_point_id].x,path_points[path_point_id].y, 45); //for smartRRT.png
		path_point_id++;
		}
		
		
		//optimize sampling area based on obstacle vertices:
		
		else if (optimize_sampling_obstacles)
		{
		
		//cout << "obstacle_cnt	" << obstacle_cnt << endl;
		//cout << "obstacle_id	" <<obstacle_id << endl;
		
		//cout << "obstacles[obstacle_id].getNbVertices()	" << obstacles[obstacle_id].getNbVertices() << endl;
		//cout << "obstacle_vertex_id	"<<obstacle_vertex_id<< endl;
		
		
		
		if (obstacle_vertex_id > obstacles[obstacle_id].getNbVertices()-1)
		{
			obstacle_vertex_id = 0;
			obstacle_id ++ ;
		}
		
		
			
		if (obstacle_id > obstacle_cnt-1)
		{
			obstacle_id = 0;
			obstacle_vertex_id = 0;
		}
		
		if (iterations%3 ==0)  //has bias toward edge but still has randomness 2 thirds of time
		{
		newPoint = pickRandomPointAround(obstacles[obstacle_id].getPoint(obstacle_vertex_id).x,obstacles[obstacle_id].getPoint(obstacle_vertex_id).y, 50); //for im_with_coord.png
		//newPoint = pickRandomPointAround(obstacles[obstacle_id].getPoint(obstacle_vertex_id).x,obstacles[obstacle_id].getPoint(obstacle_vertex_id).y, 5); //for square.PNG
		}
		else newPoint = pickRandomPoint();
		
		obstacle_vertex_id++;
		}
		
		
		//don't optimize sampling area
		else newPoint = pickRandomPoint();
		
		
		
		
		
		
		
		
		// Find nearest point to the newPoint such that the next node 
		// be added in graph in the (nearestPoint, newPoint) while being obstacle free
		nearestPoint = *nodes.begin(); nearestIndex = 0;
		for(int i = 0; i < nodeCnt; i++) {
			if(pathFound and randomCoordinate(0.0, 1.0) < 0.25) // Recalculate cost once in a while 
				cost[i] = cost[parent[i]] + distance(nodes[parent[i]], nodes[i]);  

			// Make smaller jumps sometimes to facilitate passing through narrow passages 
			jumps[i] = randomCoordinate(0.3, 1.0) * JUMP_SIZE ; 
			auto pnt = nodes[i] ; 
			if((pnt.distance(newPoint) - nearestPoint.distance(newPoint)) <= EPS and isEdgeObstacleFree(pnt, pnt.steer(newPoint, jumps[i])))
				nearestPoint = pnt, nearestIndex = i ; 
		}
		nextPoint = stepNear(nearestPoint, newPoint, jumps[nearestIndex]);
		if(!isEdgeObstacleFree(nearestPoint, nextPoint)) continue ; 


		// Find nearby nodes to the new node as center in ball of radius DISK_SIZE 
		for(int i = 0; i < nodeCnt; i++)
			if((nodes[i].distance(nextPoint) - DISK_SIZE) <= EPS and isEdgeObstacleFree(nodes[i], nextPoint))
				nearby.push_back(i);

		// Find minimum cost path to the new node 
		int par = nearestIndex; minCost = cost[par] + distance(nodes[par], nextPoint);
		for(auto nodeIndex: nearby) {
			if( ( (cost[nodeIndex] + distance(nodes[nodeIndex], nextPoint)) - minCost) <= EPS)
				minCost = cost[nodeIndex] + distance(nodes[nodeIndex], nextPoint), par = nodeIndex;
		}

		parent.push_back(par); cost.push_back(minCost);
		nodes.push_back(nextPoint); nodeCnt++; 
		updated = true ; 
		if(!pathFound) checkDestinationReached(); 
		rewire();
		
		}

		
	}
}

/////////////////////////////////////////////////////////////////////////////MAIN 

int main() {


////////////////////////////////////////////map part

Mat image = imread(image_name);
    
    const int WIDTH = image.cols ;
    const int HEIGHT = image.rows ;


    // Convert the image to grayscale
    Mat gray_image;
    cvtColor(image, gray_image, COLOR_BGR2GRAY);

    // Apply a threshold to the image to isolate the dark areas that correspond to the water bodies
    Mat thresholded_image;
  //threshold(gray_image, thresholded_image, 100, 10, THRESH_BINARY_INV); //this line is for black obstacles. remove _INV for water area (since we want dark zones to be not obstacles)// for square.png
  threshold(gray_image, thresholded_image, 40, 10, THRESH_BINARY_INV); // for M6 this line is for black obstacles. remove _INV for water area (since we want dark zones to be not obstacles)// for square.png
  //threshold(gray_image, thresholded_image, 80, 255, THRESH_BINARY); // for water area (since we want dark zones to be not obstacles)// for im_with_coord.png

    // Apply a morphological operation (erosion or dilation) to remove noise and improve the quality of the thresholded image
    Mat eroded_image;
    Mat dilated_image;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(1, 1)); // used to have no margin, for comparing with smart rrt algo on the corner of obstacle
   //Mat kernel = getStructuringElement(MORPH_RECT, Size(15, 15)); //used for both square and water, gives some margin around obstacle
    erode(thresholded_image, eroded_image, kernel);
    dilate(thresholded_image, dilated_image, kernel);

    // Find the contours of the thresholded areas
    vector<vector<cv::Point>> contours;    
    findContours(dilated_image, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); //find contour of obstacles. the result is a list of list of vertices counter clock wise 
    
    //Outputs result in terminal
    //  cout<< "number of obstacles:" << contours.size()<< endl;

/* for (int i = 0; i < contours.size(); i++ )
    {
    cout<< "obstacle #" << i << endl;
    cout<< "number of vertices:" << contours[i].size()<< endl;
    	for (int j = 0; j < contours[i].size(); j++ )
    	{
    	cout<< "vertex #" << j << "	coordinates: x=" << contours[i][j].x << "y=" << contours[i][j].y << endl;
    	}
    } */
    
    //Keep obstacles in same order while reversing the order of the vertices for each obstacle. Could save time by avoiding making a copy and directly modifying [contours]
    
    vector<vector<cv::Point>> contours_clockwise = contours;
    
    
  //reversing the order  only if not clockwise yet
    for (int i = 0; i < contours_clockwise.size(); i++ )
    {
    
    
	    int sum = 0;
	    
	    for (int c = 0; c < contours_clockwise[i].size(); c++) 
	    {
	      
		    cv::Point p1 = contours_clockwise[i][c];
		    cv::Point p2 = contours_clockwise[i][(c + 1) % contours_clockwise[i].size()];
		    sum += (p2.x - p1.x) * (p2.y + p1.y);
	    }
	  
	    if (!(sum <0))
	    {
	   	std::reverse(contours_clockwise[i].begin(), contours_clockwise[i].end() );
	    }
    
    }
    
    
    
 if (see_image_processing_only)
{
     // Draw the found contours on the original image
    Mat drawn_image = image.clone();
    
    // Draws all the obstacle at once
    drawContours(drawn_image, contours, -1, Scalar(0, 255, 0), FILLED);

	// Change color for each obstacle
    for (int i = 0; i < contours.size(); i++)
	{
	 polylines(drawn_image, contours[i], true, Scalar(0*randomCoordinate(0.0, 1.0), 250*randomCoordinate(0.0, 1.0), 0*randomCoordinate(0.0, 1.0)), 4);
}



    // Display the result
    imshow("Original Image", image);
   // imshow("Grayscale Image", gray_image);
    imshow("Thresholded Image", thresholded_image);
  //  imshow("Eroded Image", eroded_image);
    imshow("Dilated Image", dilated_image);
    imshow("Contours", drawn_image);
    waitKey(0);
    return 0;
    
}    


////////////////////////////////////////////RRT part



while (nb_simulations < nb_total_simulations)

{

path_point_id =0;
obstacle_id =0;
obstacle_vertex_id =0;
iterations = 0 ; 
iterations_after_found =0; 
nodes.clear() ; 
path_points.clear() ; 
parent.clear() ;
nearby.clear() ;
cost.clear() ;
jumps.clear() ; 
nodeCnt = 0;
goalIndex = -1 ; 
pathFound = 0 ;


auto start_time = std::chrono::high_resolution_clock::now(); //Starts measuring time here


////////////////////////////////////////////////////////

ofstream convergence_logs;
string file_name;

file_name = to_string(nb_simulations) + image_name + "_" ;
if (optimize_sampling_nodes) file_name = file_name+"optimized_";
file_name = file_name + to_string(nb_max_iterations) + ".csv";


string file_path = "../" ;
string file_instruction = file_path+file_name;

convergence_logs.open (file_instruction);
convergence_logs << "test, rrt iteration, number of waypoints, distance to goal, time\n";



if (!see_image_processing_only)
{


//start.x = 350; //for im_with_coord.png
//start.y = 300;
//stop.x = 700;
//stop.y = 100;

//start.x = 220;  //for M2.png
//start.y = 370;
//stop.x = 220;
//stop.y = 50;

//start.x = 10;  //for M3.png
//start.y = 400;
//stop.x = 400;
//stop.y = 10;

//start.x = 220;  //for M4.png
//start.y = 200;
//stop.x = 220;
//stop.y = 50;

//start.x = 130;  //for M5.png
//start.y = 10;
//stop.x = 300;
//stop.y = 400;


//start.x = 200;  //for M6.png
//start.y = 220;
//stop.x = 330;
//stop.y = 40;

//start.x = 2;  //for square.PNG
//start.y = 110;
//stop.x = 115;
//stop.y = 10;

//start.x = 10;  //for narrowpass.PNG
//start.y = 350;
//stop.x = 550;
//stop.y = 10;

//start.x = 110;  //for bugtrap.PNG
//start.y = 300;
//stop.x = 550;
//stop.y = 25;

start.x = 10;  //for complex_env.PNG
start.y = 440;
stop.x = 590;
stop.y = 10;

//start.x = 119;  //for smartRRTtest.png
//start.y = 111;
//stop.x = 197;
//stop.y = 5;

//start.x = 1000;  //for MIT_SP.tif (in pixel)
//start.y = 1500;
//stop.x = 3000;
//stop.y = 1500;

/*double goal_lat = 42.358258; //for MIT_SP.tif going around the dock
double goal_lon = -71.088378;
double start_lat = 42.358738;
double start_lon = -71.087108;

start.x = convert_latlon_to_pixels(start_lat, start_lon).x;
start.y = convert_latlon_to_pixels(start_lat, start_lon).y+10; //-10; //adjusting by 10 pixels to not be against the dock.
stop.x = convert_latlon_to_pixels(goal_lat, goal_lon).x-10;
stop.y = convert_latlon_to_pixels(goal_lat, goal_lon).y-10; //+10; //adjusting by 10 pixels to not be against the dock.

cout << start.x << "	" << start.y << "	"<< stop.x << "	" << stop.y << endl; */


//pnts //nb of vertices of an obstacle. the vertices then have to be given in clockwise order. we get a counter clockwise from the map reader. how to solve this? : use reverse() on the vertex list
//pnt //vertex of an obstacle

//*  was the get input function, basically replaced by info from map part to create list of obstacles (obstacles = contours_clockwise;)

///*

obstacle_cnt = contours.size(); //nb of obstacles from map part (devrait etre contours_clockwise? surement pareil)
obstacles.resize(obstacle_cnt); 
int pnts = 0 ; 
MyLib::GeometryPoint pnt ; 
vector < MyLib::GeometryPoint > poly ; 
	
	for(int i = 0; i < obstacle_cnt; i++) {
		poly.clear();
		pnts = contours_clockwise[i].size(); 
		poly.resize(pnts);

		for(int j = 0; j < pnts; j++) {

			pnt.x = contours_clockwise[i][j].x;
			pnt.y = contours_clockwise[i][j].y;
			obstacles[i].addPoint(pnt);
		}
	}



	

	
	prepareInput(); 
	
	
    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Basic Anytime RRT");
    
    sf::View view(sf::FloatRect(0, 0, WIDTH, HEIGHT)); //add resizable 

	nodeCnt = 1; nodes.push_back(start); iterations = 0 ; iterations_after_found =0;
	parent.push_back(0); cost.push_back(0);
    sf::Time delayTime = sf::milliseconds(5);
    
    /////trying to load the image 
    sf::Texture texture;
    if (!texture.loadFromFile(image_name))
        return -1;

    // Create a sprite to display the image
    sf::Sprite sprite(texture);
    
    

   // cout << endl << "Starting node is in Pink and Destination node is in Blue" << endl << endl ; 
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
            	window.close();
            	return 0; exit(0);
            }
            
            else if (event.type == sf::Event::Resized) {
                // Resize and re-center the view to keep the aspect ratio
                resizeView(window, view);

                // Resize the window back to maintain the aspect ratio, if the difference is significant
                auto size = window.getSize();
                sf::Vector2u newSize;

                if (size.x / static_cast<float>(WIDTH) > size.y / static_cast<float>(HEIGHT)) {
                    newSize.x = static_cast<unsigned int>(size.y * (static_cast<float>(WIDTH) / HEIGHT));
                    newSize.y = size.y;
                } else {
                    newSize.x = size.x;
                    newSize.y = static_cast<unsigned int>(size.x * (static_cast<float>(HEIGHT) / WIDTH));
                }

                // Only resize if the size difference is more than a few pixels
                if (abs(static_cast<int>(newSize.x - size.x)) > 3 || abs(static_cast<int>(newSize.y - size.y)) > 3) {
                    window.setSize(newSize);
                }
            }
        }
        RRT(); iterations++;
        
        
        
        if(pathFound) 
        {
       // if(iterations_after_found==0)
       // {
        auto end_time = std::chrono::high_resolution_clock::now();

		    // calculate and print the duration
		    auto duration_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
		   // std::cout << "Found after: " << duration_time/1000.0 << " seconds." << std::endl;
       // }
        iterations_after_found++; 
        convergence_logs << "1," << iterations << "," << path_points.size() << "," << cost[goalIndex] << "," << duration_time << "\n"; //write the distance only after found (to check convergence only)
        }
        
        //convergence_logs << "1," << iterations << "," << path_points.size() << "," << cost[goalIndex] << "\n"; //write all the distance for all iterations
        
		if(iterations % 500 == 0) {
			cout << "Iterations: " << iterations << endl ; 
			cout << "Iterations after found: " << iterations_after_found << endl ; 
			
			if(!pathFound) cout << "Not reached yet :( " << endl ;
			else cout << "Shortest distance till now: " << cost[goalIndex] << " units." << endl ;
			cout << endl ;
		}
		
		//if (iterations_after_found == nb_max_iterations_after_found)
		if (iterations == nb_max_iterations)
		{
		
		
		
		auto end_time = std::chrono::high_resolution_clock::now();

		    // calculate and print the duration
		    auto duration_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
		    std::cout << "Duration: " << duration_time/1000.0 << " seconds." << std::endl;

		
		
		 //converting pixels to long and lat of the forrest tif file and writing a plug file 
		
		
		
		ofstream myfile;
		  myfile.open ("../../../moos-ivp-tschaefe/missions/m2_berta_blackout_charles/plug_test.bhv");
		  myfile << "//----------------------------------------------.\n";
		  myfile << "Behavior = BHV_Waypoint  \n";
		  myfile << "{  \n";
		  myfile << "name      = waypt_return  \n";
		  myfile << "pwt       = 100  \n";
		  myfile << "updates   = RETURN_UPDATES  \n";
		  myfile << "condition = MODE==RETURNING  \n";
		  myfile << "endflag   = STATION_KEEP = true  \n";
		  myfile << "endflag   = AVOID = false  \n";
		  myfile << "speed = 1.3  \n";
		  myfile << "radius = 3.0  \n";
		  myfile << "nm_radius = 15.0 \n";
		  myfile << "points = ";
		  
		  if (path_points.size() != 0)
		  {
		  
		  for (int l = 0; l <path_points.size()-1; l++ )
		  
		  {
		  
		  path_points[l] = convert_to_pmarineviewer(path_points[l]);
		  
		  myfile << path_points[l].x << "," << path_points[l].y << " : " ;
		  }
		  
		  path_points[path_points.size()-1] = convert_to_pmarineviewer(path_points[path_points.size()-1]);
		  
		  myfile << path_points[path_points.size()-1].x << "," << path_points[path_points.size()-1].y;
		  myfile << " \n";
		  myfile << "order = reverse  \n";
		  myfile << "lead = 8  \n";
		  myfile << "} \n";
		  
		  }
		  
		  
		  myfile.close();
		  convergence_logs.close();
		  
		 // return 0;  uncomment to come back
		 nb_simulations++; //to remove to come back
		 window.close();
		}
		
		//cout << obstacle_cnt;

		
		//window.clear();
		//window.draw(sprite);
		//draw(window); 
		
		
		
        //window.display();
        
        	window.clear();
        	window.setView(view);
        	// Draw your elements here
        	window.draw(sprite);
        	draw(window);
        	window.display();
    }
}

}    
 //   */
//waitKey(0);
return 0;
    
}



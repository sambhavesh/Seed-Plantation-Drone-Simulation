//compile with : g++ --std=c++11 gen.cpp
//execute with : ./a.out >output.txt
#include <bits/stdc++.h>
using namespace std;
typedef long long int ll;
typedef long double dd;
#define INF 1e12 + 5 
  
struct Point 
{ 
    dd x; 
    dd y; 
};  
bool onSegment(Point p, Point q, Point r) 
{ 
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y)) return true; 
    return false; 
} 
int orientation(Point p, Point q, Point r) 
{ 
    dd val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y); 
  
    if (val == 0) return 0;  // colinear 
    return (val > 0)? 1: 2; // clock or counterclock wise 
} 
bool doIntersect(Point p1, Point q1, Point p2, Point q2) 
{ 
    int o1 = orientation(p1, q1, p2); 
    int o2 = orientation(p1, q1, q2); 
    int o3 = orientation(p2, q2, p1); 
    int o4 = orientation(p2, q2, q1);  
    if (o1 != o2 && o3 != o4) return true;  
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;  
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;  
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;  
    if (o4 == 0 && onSegment(p2, q1, q2)) return true; 
  
    return false; 
} 
  
//function to check if point is inside
bool isInside(vector<Point> &polygon, Point p) 
{ 
	ll n=polygon.size(); 
    if (n < 3)  return false; 
  
    // Create a point for line segment from p to infinity 
    Point extreme = {INF, p.y}; 
  
    // Count intersections of the above line with sides of polygon 
    ll count = 0, i = 0; 
    do
    { 
        ll next = (i+1)%n; 
  
        // Check if the line segment from 'p' to 'extreme' intersects 
        // with the line segment from 'polygon[i]' to 'polygon[next]' 
        if (doIntersect(polygon[i], polygon[next], p, extreme)) 
        { 
            // If the point 'p' is colinear with line segment 'i-next', 
            // then check if it lies on segment. If it lies, return true, 
            // otherwise false 
            if (orientation(polygon[i], p, polygon[next]) == 0) 
               return onSegment(polygon[i], p, polygon[next]); 
  
            count++; 
        } 
        i = next; 
    } while (i != 0); 
  
    // Return true if count is odd, false otherwise 
    return count&1;  
} 

dd max(dd a, dd b)
{
	if(a>b)return a;
	else return b;
}
//write function(list,d)
void generate_points(vector<Point> &polygon, dd d, dd err)
{
	ll iter=0;
	ll n=polygon.size();
	dd xmin=polygon[0].x;
	dd ymin=polygon[0].y;
	dd xmax=xmin;
	for(ll i=1;i<n;i++)
	{
		xmax=max(xmax,polygon[i].x);
		if(polygon[i].x<xmin-err)
		{
			xmin=polygon[i].x;
			ymin=polygon[i].y;
			// iter=i;
		}
		else if(polygon[i].x<=xmin+err && polygon[i].x>=xmin-err)
		{
			if(polygon[i].y<ymin-err)
			{
				xmin=polygon[i].x;
				ymin=polygon[i].y;
				// iter=i;
			}
		}
	}
	
	cout<<xmin<<" "<<ymin<<endl;
	bool switch1 =false, switch2 = true;
	Point tmp;
	tmp.x=xmin;
	tmp.y=ymin;
	while(tmp.x<=xmax)
	{
		if(isInside(polygon,tmp))
		{
			switch1=true;
		}
		else
		{

			switch1=false;
		}
		if(switch1==false)
		{
			tmp.x+=d;
			switch2=!switch2;
		}
		
		if(switch2==true)
		{
			tmp.y+=d;
		}
		else
		{
			tmp.y-=d;
		}
		cout<<tmp.x<<" "<<tmp.y<<endl;
	}
}

int main()
{
	ios_base::sync_with_stdio(false);
	cin.tie(NULL);
	cout.tie(NULL);
	//Test polygon
	vector<Point> polygon;
	polygon.push_back({0.0,1.0});
	polygon.push_back({1.0,2.0});
	polygon.push_back({3.0,1.0});
	polygon.push_back({3.0,0.0});
	polygon.push_back({0.0,0.0});
	generate_points(polygon, 0.2, 0.0);
}

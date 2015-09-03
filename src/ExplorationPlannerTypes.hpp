#ifndef EXPLORATION_TYPES_H
#define EXPLORATION_TYPES_H

#include <vector>
#include <base/Eigen.hpp>

namespace exploration
{	
    enum Status {SUCCESS, NO_GOAL, ERROR, NOT_IMPLEMENTED};            
    enum DrivabilityClasses {VISIBLE, OBSTACLE = 2, EXPLORED, UNKNOWN };
    
	struct GridPoint
	{
        GridPoint() : x(0), y(0), distance(0) {
        }
        
        GridPoint(int x_, int y_, int distance_) : x(x_), y(y_), distance(distance_) {
        }
        
		int x;
		int y;
		int distance;
	};

	struct FloatPoint
	{
        FloatPoint() : x(0.0), y(0.0) {
        }
        
		double x;
		double y;
	};

	struct Pose
	{
        Pose() : x(0.0), y(0.0), theta(0.0) {
        }
        
		double x;
		double y;
		double theta;
	};
        
    //used since std::vector<std::vector<FloatPoint>> doesn't seem to work with oroGen configs
    struct ConfPolygon
    {
        std::vector<FloatPoint> points;
    };

	typedef std::vector<GridPoint> PointList;
	typedef std::vector<PointList> FrontierList;
	typedef std::vector<FloatPoint> Polygon;
	typedef std::vector<Polygon> SensorField;

	class GridMap
	{
	public:
		GridMap(unsigned int w, unsigned int h)
		{
			width = w;
			height = h;
			data = (char*)calloc(w*h, sizeof(char));
            memset(data, UNKNOWN, sizeof(char)*w*h);
			isAllocated = true;
		}

		GridMap(unsigned int w, unsigned int h, char* d)
		{
			width = w;
			height = h;
			data = d;
			isAllocated = false;
		}

		~GridMap()
		{
			if(isAllocated) delete[] data;
		}

		bool getData(GridPoint p, char& c) const
		{
			if(p.x < 0 || p.y < 0 || p.x >= (int)width || p.y >= (int)height)
				return false;
			else {
				c = data[(p.y*width)+p.x];
                return true;
            }
		}

		bool setData(GridPoint p, char v)
		{
			if(!isAllocated || p.x < 0 || p.y < 0 || 
                    p.x >= (int)width || p.y >= (int)height) {
                return false;
            }
			data[(p.y*width)+p.x] = v;
			return true;
		}

		char* getData() {
            return data;
        }
        
		bool setData(unsigned int index, char v)
		{
			if(!isAllocated || index >= width * height) {
                return false;
            }
			data[index] = v;
			return true;
		}
		
		unsigned int getWidth() const {
            return width;
        }
        
		unsigned int getHeight() const {
            return height;
        }

	private:
		char* data;
		unsigned int width;
		unsigned int height;
		bool isAllocated;
	};
}

#endif

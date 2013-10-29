#ifndef EXPLORATION_TYPES_H
#define EXPLORATION_TYPES_H

#include <vector>

namespace exploration
{	
	struct GridPoint
	{
		unsigned int x;
		unsigned int y;
		int distance;
	};

	struct FloatPoint
	{
		double x;
		double y;
	};

	struct Pose
	{
		double x;
		double y;
		double theta;
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
			data = new char[w*h];
			for(unsigned int i = 0; i < w*h; i++) data[i] = -1;
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

		char getData(GridPoint p)
		{
			if(p.x >= width || p.y >= height)
				return -1;
			else
				return data[(p.y*width)+p.x];
		}

		bool setData(GridPoint p, char v)
		{
			if(!isAllocated || p.x >= width || p.y >= height) return false;
			data[(p.y*width)+p.x] = v;
			return true;
		}

		char* getData() {return data;}
		bool setData(unsigned int index, char v)
		{
			if(!isAllocated || index >= width * height) return false;
			data[index] = v;
			return true;
		}
		
		unsigned int getWidth() {return width;}
		unsigned int getHeight() {return height;}

	private:
		char* data;
		unsigned int width;
		unsigned int height;
		bool isAllocated;
	};
}

#endif

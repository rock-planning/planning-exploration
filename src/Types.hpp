#ifndef EXPLORATION_TYPES_H
#define EXPLORATION_TYPES_H

namespace exploration
{
	struct GridPoint
	{
		unsigned int x;
		unsigned int y;
	};

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

		bool getData(GridPoint p, char &v)
		{
			if(p.x >= width || p.y >= height) return false;
			v = data[(p.y*width)+p.x];
			return true;
		}

		bool setData(GridPoint p, char v)
		{
			if(!isAllocated || p.x >= width || p.y >= height) return false;
			data[(p.y*width)+p.x] = v;
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

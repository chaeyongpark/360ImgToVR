//
//  segmentation.h
//  
//
//  Created by 황일환 on 2016. 10. 23..
//
//

#ifndef ____segmentation__
#define ____segmentation__

#include <stdio.h>
#include <vector>
#include <set>
#include <opencv2/opencv.hpp>

struct SegComponent;
struct SegEdge;

struct SegVertex {
    int x, y;
	float w, theta, phi;
    cv::Vec3f color;
	SegComponent* container;
	SegVertex() : x(-1), y(-1), color(cv::Vec3f(0, 0, 0)), container(nullptr) {}
	SegVertex(int x, int y, float w, cv::Vec3f color, float theta, float phi) : x(x), y(y), w(w), color(color), theta(theta), phi(phi) {}
};

struct SegEdge {
    float w;
	SegVertex *v1, *v2;
	SegEdge() : w(0.0f), v1(nullptr), v2(nullptr) {}
	SegEdge(SegVertex *v1, SegVertex *v2, float metric) : v1(v1), v2(v2) {
		w = 0.0;
		w += (v1->color[0] - v2->color[0]) * (v1->color[0] - v2->color[0]);
		w += (v1->color[1] - v2->color[1]) * (v1->color[1] - v2->color[1]);
		w += (v1->color[2] - v2->color[2]) * (v1->color[2] - v2->color[2]);
		w = sqrt(w);
	};
    bool operator< (const SegEdge& other) {
        return w < other.w;
    }
    bool operator> (const SegEdge& other) {
        return w > other.w;
    }
};

struct SegComponent {
    int id;
    std::vector<SegVertex*> vertices;
    float internal, magnitude;
	SegComponent(int id) : internal(0.0f), magnitude(0.0f), id(id) {}
};

struct lessComp {
    bool operator() (const SegEdge& e1, const SegEdge& e2) const {
        return e1.w < e2.w;
    }
    bool operator() (const SegEdge *e1, const SegEdge *e2) const {
        return e1->w < e2->w;
    }
    bool operator() (const SegComponent *c1, const SegComponent *c2) const {
        return c1->id < c2->id;
    }
};

struct greatComp {
    bool operator() (const SegEdge& e1, const SegEdge& e2) const {
        return e1.w > e2.w;
    }
    bool operator() (const SegEdge *e1, const SegEdge *e2) const {
        return e1->w > e2->w;
    }
};

class Segmentation {
    std::vector<SegVertex> vertices;
    std::vector<SegEdge> edges;
    size_t width, height;
    std::set<SegComponent*, lessComp> components;
    float k;
	int regionThreshold;
	float vertexSum;
	cv::Mat img_origin;

	SegVertex *vertex(int x, int y) { return &vertices[x + y * width]; }
    float mint(SegComponent *c1, SegComponent *c2);
	void merge(SegComponent *c1, SegComponent *c2, float w);
    
public:
    //Segmentation();
	void init(cv::Mat img, float k, float threshold);
	void build();
    void output(cv::Mat img);
	std::set<SegComponent*, lessComp>& getComponents() { return components; }
	float getVertexSum() { return vertexSum; }
	cv::Size getSize() { return cv::Size(width, height); }
	int findObject(cv::Mat img);
};

#endif /* defined(____segmentation__) */

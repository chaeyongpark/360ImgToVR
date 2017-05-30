//
//  segmentation.cpp
//  
//
//  Created by 황일환 on 2016. 10. 23..
//  Modified byy 박채용 on 2017.05
//
//

#include "segmentation.h"
#include <algorithm>
#include <cmath>
#include <queue>

static const float pi = 3.14159265;
using namespace std;

void Segmentation::init(cv::Mat img, float k, float threshold) {
	this->k = k;
	width = img.size().width;
    height = img.size().height;
    
	regionThreshold = (int)(threshold * width * height);
	img_origin = cv::Mat::zeros(img.size().height, img.size().width, CV_8UC4);
	img_origin = img;
	
		
	cout << "HI" << endl;
    int cid = 0;
    
	vertexSum = 0.0;

    for (int y = 0; y < height; y++) {
		const float w = sin((float)(y + 0.5) / height * pi);
		float phi = -(float)(y - (int)height / 2) / height * pi;
		vertexSum += w;
        for (int x = 0; x < width; x++) {
			cv::Vec3f col = img.at<cv::Vec3b>(y, x);
			float theta = (float)x / width * 2.0f * pi;
            vertices.push_back(SegVertex(x, y, w, col, theta, phi));
        }
    }
	cout << "Initializing Segmentation..." << endl;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
			SegVertex *v = vertex(x, y);
			SegComponent *c = new SegComponent(cid++);
            components.insert(c);
            v->container = c;
            c->vertices.push_back(v);
            c->internal = 0.0;
			c->magnitude = v->w;
        }
    }
	//Horizontal edges
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width - 1; x++) {
            edges.push_back(SegEdge(vertex(x, y), vertex(x + 1, y), vertex(x, y)->w));
        }
    }
	//Rolling back case
	for (int y = 0; y < height; y++) {
		edges.push_back(SegEdge(vertex(0, y), vertex(width - 1, y), vertex(0, y)->w));
	}
	//Vertical edges
    for (int y = 0; y < height - 1; y++) {
        for (int x = 0; x < width; x++) {
            edges.push_back(SegEdge(vertex(x, y), vertex(x, y + 1), 1.0f));
        }
    }
	//Diagonal edges
    for (int y = 0; y < height - 1; y++) {
		const float w = sqrt(vertex(0, y)->w * vertex(0, y)->w + 1.0f);
        for (int x = 0; x < width - 1; x++) {
            edges.push_back(SegEdge(vertex(x, y), vertex(x + 1, y + 1), w));
            edges.push_back(SegEdge(vertex(x + 1, y), vertex(x, y + 1), w));
        }
    }
	//Rolling back case
	for (int y = 0; y < height - 1; y++) {
		const float w = sqrt(vertex(0, y)->w * vertex(0, y)->w + 1.0f);
		edges.push_back(SegEdge(vertex(0, y), vertex(width - 1, y + 1), w));
		edges.push_back(SegEdge(vertex(width - 1, y), vertex(0, y + 1), w));
	}
	cout << "Initialization Finish." << endl;
}

float Segmentation::mint(SegComponent *c1, SegComponent *c2) {
    return fmin(c1->internal + k / c1->magnitude, c2->internal + k / c2->magnitude);
}

void Segmentation::merge(SegComponent *c1, SegComponent *c2, float w) {
	for (int i = 0; i < c2->vertices.size(); i++)
		c2->vertices[i]->container = c1;

	c1->magnitude += c2->magnitude;
	c1->vertices.insert(c1->vertices.end(), c2->vertices.begin(), c2->vertices.end());
	components.erase(components.find(c2));

	c1->internal = fmax(c2->internal, c1->internal);
	c1->internal = fmax(w, c1->internal);

	delete c2;
}

void Segmentation::build() {
	cout << "building.. edge size:" << edges.size() << endl;
    std::vector<SegEdge*> pedges;
    for (int i = 0; i < edges.size(); i++)
        pedges.push_back(&edges[i]);
    
    std::sort(pedges.begin(), pedges.end(), lessComp());
    
    for (int q = 0; q < pedges.size(); q++) {
		SegEdge *o = pedges[q];
		SegVertex *v1 = o->v1;
		SegVertex *v2 = o->v2;
		SegComponent *c1 = v1->container;
		SegComponent *c2 = v2->container;
        
        if (c1 == c2)
            continue;
        
        if (o->w <= mint(c1, c2)) {
			merge(c1, c2, o->w);
        }
    }
	for (int q = 0; q < pedges.size(); q++) {
		SegEdge *o = pedges[q];
		SegVertex *v1 = o->v1;
		SegVertex *v2 = o->v2;
		SegComponent *c1 = v1->container;
		SegComponent *c2 = v2->container;

		if (c1 == c2)
			continue;

		if (c1->vertices.size() < regionThreshold || c2->vertices.size() < regionThreshold) {
			merge(c1, c2, o->w);
		}
	}
	cout << "building finish" << endl;
}

void Segmentation::output(cv::Mat img) {
	int cnt = 0;
	cv::Mat tmp_img = cv::Mat::zeros(img.size().height, img.size().width, CV_8UC3);

	std::vector<SegVertex*> vs;
    std::set<SegComponent*, lessComp>::iterator itor;

	cout << "Starting Segmentation..." << endl;
    for (itor = components.begin(); itor != components.end(); itor++) {
		vs = (*itor)->vertices;
		cv::Vec3b color = cv::Vec3b((unsigned char)cnt, (unsigned char)(rand() % 255), (unsigned char)(rand() % 255));

        for (int i = 0; i < vs.size(); i++) {
			SegVertex& v = *vs[i];
			tmp_img.at<cv::Vec3b>(v.y, v.x) = color;
        }
		cnt++;
    }
	cv::imshow("Tmp Image", tmp_img);

	int max_component = findObject(tmp_img);

	cnt = 0;
	for (itor = components.begin(); itor != components.end(); itor++) {
		if (cnt == max_component) {
			vs = (*itor)->vertices;
			for (int i = 0; i < vs.size(); i++) {
				SegVertex& v = *vs[i];
				img.at<cv::Vec4b>(v.y, v.x) = cv::Vec4b(img_origin.at<cv::Vec3b>(v.y, v.x)[0],
														img_origin.at<cv::Vec3b>(v.y, v.x)[1],
														img_origin.at<cv::Vec3b>(v.y, v.x)[2],
														(unsigned char)255);
			}
		}
		cnt++;
	}
	cout << "Finish Segmentation" << endl;
}

int Segmentation::findObject(cv::Mat img) {
	const cv::Point2f start_pnt = { (float)img.size().width / 20 * 9, (float)img.size().height / 20 * 9 };
	const cv::Point2f end_pnt = { (float)img.size().width / 20 * 11, (float)img.size().height / 20 * 11 };
	
	int arr[256] = { 0 };
	int max = -1;
	int max_component;

	for (int i = start_pnt.x; i < end_pnt.x; i++) {
		for (int j = start_pnt.y; j < end_pnt.y; j++) {
			arr[(int)img.at<cv::Vec3b>(j, i)[0]]++;
		}
	}
	for (int i = 0; i < 256; i++) {		
		if (arr[i] > max) {
			max_component = i;
			max = arr[i];
		}
	}

	return max_component;
}
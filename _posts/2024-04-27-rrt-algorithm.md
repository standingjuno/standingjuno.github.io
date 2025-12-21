---
layout: post
title: rrt algorithm (c++)
tags: [path_planning]
---

<h1>RRT Algorithm</h1>

RRT(Rapidly-exploring Random Tree) 알고리즘은 수많은 경로계획(Path Planning) 알고리즘 중 하나로 샘플링 기반의 경로계획 알고리즘이다.

RRT는 풀네임에서 알 수 있듯이 Tree 자료구조를 사용하며 시작지점을 root로 하는 Tree라고 생각하면 된다.

Random Point를 생성하여 시작지점(Start Point)에서 목표지점(Destination Point)까지 Tree를 확장하여 도달하는 알고리즘이라고 할 수 있다.

아래는 RRT 알고리즘의 단계를 쉽게 이해되도록 설명한 것이다. (쉽게 설명하고자 노력하였음...)

1. 새로운 Random Point를 생성한다.
2. 생성된 Random Point는 Tree에서 가장 가까운 노드를 찾고, jump 길이만큼 New Point를 생성한다. (jump는 사람이 정하는 parameter, jump가 길수록 목적지에 빠르게 도착할 수 있음)
3. New Point는 2번 과정에서 찾은 Tree에서 가장 가까운 노드를 부모노드(Parent Node)로 갖고, 기존의 Tree로 편입된다.
4. New Point가 장애물 안에 위치한다면 경로로 인정될 수 없으니 New Point를 Tree에서 제거한다.
5. New Point와 부모노드를 이은 경로(Branch)가 장애물과 겹친다면 경로로 인정될 수 없으니 New Point를 Tree에서 제거한다.
6. 1~5번 과정을 도착지점에 도달할 때까지 반복한다.

---
조금 더 쉽게 이해할 수 있도록 그림으로 보자.

먼저 이렇게 생긴 맵이 있다고 가정하자. 빨간색은 시작지점, 파란색은 도착지점, 보라색은 장애물이다. 오른쪽은 트리구조를 나타낸 것이다.
![rrt1](https://github.com/its-seon/its-seon.github.io/assets/145862553/4a1e8271-e089-452a-a9dd-cb8db1bf6ddd)

Random Point를 생성한다.
![rrt2](https://github.com/its-seon/its-seon.github.io/assets/145862553/940a14aa-0151-4516-af86-398754145ca3)

1번과 가장 가까운 노드를 찾고 jump 길이만큼 떨어진 곳에 1번 New Point를 생성한다. 1번은 start와 가장 가깝기 때문에 start를 부모노드로 갖는다.
![rrt3](https://github.com/its-seon/its-seon.github.io/assets/145862553/31e5e218-f378-4701-adb7-236535abaad4)

1번 New Point는 장애물과 겹치지 않기 때문에 트리에 편입시킨다.
![rrt4](https://github.com/its-seon/its-seon.github.io/assets/145862553/d2398edf-373f-46e3-a4eb-ab057d4e9fa4)

(이제부터는 jump길이만큼 Random Point가 생성된다고 가정한다. 즉 Random Point를 New Point라고 생각하자.)

마찬가지로 2번 New Point를 생성하고 2번과 가장 가까운 노드인 1번노드를 부모노드로 갖도록 트리에 편입시킨다.
![rrt5](https://github.com/its-seon/its-seon.github.io/assets/145862553/91c5924c-89c3-4c05-a42c-4c0c4d83706e)

그러나 2번 New Point는 장애물과 겹쳐 경로가 될 수 없기 때문에 트리에서 제거한다.
![rrt6](https://github.com/its-seon/its-seon.github.io/assets/145862553/9eceeac4-064a-4f34-a8c2-f2a1450e7a86)

3번 New Point를 생성한다. 3번 New Point와 가장 가까운 노드를 트리에서 찾으면 start가 가장 가깝기 때문에 start를 부모노드로 갖도록 편입시킨다. (장애물과 겹치지 않음)
![rrt7](https://github.com/its-seon/its-seon.github.io/assets/145862553/f0980efc-94ab-456a-8dcf-206bcc1b94eb)

4번 New Point를 생성한다. 4번과 가장 가까운 노드를 트리에서 찾을 때 3번이 가장 가깝기 때문에 3번을 부모노드로 갖도록 편입시킨다. (장애물과 겹치지 않음)
![rrt8](https://github.com/its-seon/its-seon.github.io/assets/145862553/63c6c506-36b1-472b-8d7d-a9562b24142e)

이런식으로 반복하다보면 장애물과 겹치는 Point들은 트리에 추가될 수 없으며 언젠가는 목적지에 도달하는 경로가 생성될 수 있다.
![rrt9](https://github.com/its-seon/its-seon.github.io/assets/145862553/40f2a400-3e0e-4aed-9718-f35183da6225)

---
<h3>단점</h3>

RRT 알고리즘은 아주 유명한 경로계획법이지만, 여러가지 단점도 존재한다.
<ul>
<li>샘플링 개수가 충분하지 않다면 경로를 찾지 못할 수 있다. (모든 샘플링 기반 경로계획법의 단점)</li>
<li>경로를 찾더라도 최적(optimal) 경로가 아니다.</li>
<li>도착점까지 수렴하는데 많은 시간이 소요되며, 예측이 불가능하다.</li>
<li>산출된 경로가 랜덤 샘플링의 성격상 완만하지 않아 실제 로봇이 그대로 주행하기는 어려울 수 있다. (갑자기 90도로 꺽는 곳에서는 자동차는 주행할 수 없음)</li>
</ul>

---
<h3>요약</h3>

RRT 알고리즘은 유명한 랜덤 샘플링 경로계획법으로 실제 로봇 소프트웨어에서 많이 사용되는 알고리즘이다. 기존의 RRT 알고리즘의 단점을 보완하기 위해 RRT*와 같은 알고리즘도 나왔으며 계속적으로 연구가 되는 것을 알 수 있다.

---
<h3>Reference</h3>

<https://pasus.tistory.com/74>

---
<h3>c++ 구현</h3>

```cpp
#include <iostream>
#include <random>
#include <vector>
#include <cmath>
#include "opencv2/opencv.hpp"

using namespace std;

struct Point {
    double x;
    double y;
    int parentIndex = -1;
    Point(double x = 0, double y = 0) : x(x), y(y) {}
};

struct Obstacle {
    Point leftTop;
    Point rightBottom;
    Obstacle(const Point& lt, const Point& rb) : leftTop(lt), rightBottom(rb) {}
};

const int width = 800;
const int height = 800;
const int jump = 30;

vector<Point> path;
vector<Obstacle> obstacles;

Point newPoint;

/* start and destionation setup */
Point start;
Point destination;

void initRRT(int startX, int startY, int endX, int endY) {
    start = Point(startX, startY);
    destination = Point(endX, endY);

    path.clear();
    path.push_back(start);
}

/* create obstacles */
void initObstacles() {
    obstacles.clear();
    obstacles.push_back(Obstacle({100, 100}, {200, 200}));
    obstacles.push_back(Obstacle({300, 300}, {400, 400}));
    obstacles.push_back(Obstacle({100, 300}, {200, 400}));
}

/* generate random point */
random_device rd;
mt19937 gen(rd());
uniform_int_distribution<> dis_x(0, width);
uniform_int_distribution<> dis_y(0, height);

Point generateRandomPoint() {
    return Point(dis_x(gen), dis_y(gen));
}

/* find the nearest point */
Point& nearestPoint(const Point& randomPoint) {
    double minDistance = numeric_limits<double>::max();
    int nearestIndex = 0;

    for (int i = 0; i < path.size(); i++) {
        double distance = sqrt(pow(path[i].x - randomPoint.x, 2) + pow(path[i].y - randomPoint.y, 2));
        if (distance < minDistance) {
            minDistance = distance;
            nearestIndex = i;
        }
    }
    return path[nearestIndex];
}

/* check if the random point is within the obstacles */
bool isInObstacle(const Point& p) {
    for (const auto& obs : obstacles) {
        if (p.x > obs.leftTop.x && p.x < obs.rightBottom.x &&
            p.y > obs.leftTop.y && p.y < obs.rightBottom.y) {
            return true;
        }
    }
    return false;
}

/* check if the path intersects with the obstacles */
bool checkPathIntersectionWithObstacles(Point& nearest, Point& newPoint){
    for (const auto& obs : obstacles) {
        Point topRight = {obs.rightBottom.x, obs.leftTop.y};
        Point bottomLeft = {obs.leftTop.x, obs.rightBottom.y};

        std::vector<std::pair<Point, Point>> edges = {
            {obs.leftTop, topRight},
            {topRight, obs.rightBottom},
            {obs.rightBottom, bottomLeft},
            {bottomLeft, obs.leftTop}
        };

        for (const auto& edge : edges) {
            double o1 = (newPoint.y - nearest.y) * (edge.first.x - nearest.x) - (newPoint.x - nearest.x) * (edge.first.y - nearest.y);
            double o2 = (newPoint.y - nearest.y) * (edge.second.x - nearest.x) - (newPoint.x - nearest.x) * (edge.second.y - nearest.y);
            double o3 = (edge.second.y - edge.first.y) * (nearest.x - edge.first.x) - (edge.second.x - edge.first.x) * (nearest.y - edge.first.y);
            double o4 = (edge.second.y - edge.first.y) * (newPoint.x - edge.first.x) - (edge.second.x - edge.first.x) * (newPoint.y - edge.first.y);

            if ((o1 * o2 < 0) && (o3 * o4 < 0)) {
                return true;
            }
        }
    }
    return false;
}

/* generate path */
void generatePath() {
    Point randomPoint = generateRandomPoint();
    Point& nearest = nearestPoint(randomPoint);

    double angle = atan2(randomPoint.y - nearest.y, randomPoint.x - nearest.x);
    newPoint = Point(nearest.x + jump * cos(angle), nearest.y + jump * sin(angle));
    cout << "New point: (" << newPoint.x << ", " << newPoint.y << ")" << endl;
    
    if (!isInObstacle(newPoint) && !checkPathIntersectionWithObstacles(nearest, newPoint)) {
        newPoint.parentIndex = &nearest - &path[0];
        path.push_back(newPoint);
    }

    if (sqrt(pow(path.back().x - destination.x, 2) + pow(path.back().y - destination.y, 2)) < jump) {
        path.push_back(destination);
        cout << "Destination reached." << endl;
    }
}

void updateVisualization(cv::Mat& img) {
    img.setTo(cv::Scalar(0, 0, 0));

    for (const auto& obs : obstacles) {
        cv::rectangle(img, cv::Point(obs.leftTop.x, obs.leftTop.y), cv::Point(obs.rightBottom.x, obs.rightBottom.y), cv::Scalar(0, 0, 255), -1);
    }

    // Draw lines from each point to its parent
    for (size_t i = 0; i < path.size(); i++) {
        if (path[i].parentIndex != -1) {
            cv::line(img, cv::Point(path[i].x, path[i].y), cv::Point(path[path[i].parentIndex].x, path[path[i].parentIndex].y), cv::Scalar(255, 255, 255), 2);
        }
        else{
            cv::line(img, cv::Point(path[i-1].x, path[i-1].y), cv::Point(path[i].x, path[i].y), cv::Scalar(255, 255, 255), 2);
        }
    }


    cv::circle(img, cv::Point(start.x, start.y), 5, cv::Scalar(0, 255, 0), cv::FILLED); // Green for start
    cv::circle(img, cv::Point(destination.x, destination.y), 5, cv::Scalar(255, 0, 0), cv::FILLED); // Blue for destination
    cv::circle(img, cv::Point(newPoint.x, newPoint.y), 5, cv::Scalar(0, 165, 255), cv::FILLED);  // Orange for newPoint

    cv::imshow("RRT Path Planning", img);
    cv::waitKey(20); // Short delay to update the display
}

int main() {
    initRRT(30, 30, 770, 770); // startX, startY, endX, endY point
    initObstacles(); // Initialize obstacles

    int iterations = 0;
    const int maxIterations = 50000; // Limit the number of iterations to avoid infinite loop

    cv::Mat img(width, height, CV_8UC3, cv::Scalar(0, 0, 0));

    while (true) {
        if (iterations++ > maxIterations) {
            cout << "Max iterations reached." << endl;
            break;
        }

        generatePath();
        updateVisualization(img);

        if (sqrt(pow(path.back().x - destination.x, 2) + pow(path.back().y - destination.y, 2)) < jump) {
            break;
        }
    }

    cv::waitKey(0); // Wait for a key press
    cout << "Path generated with " << path.size() << " points." << endl;

    return 0;
}
```

---
<h3>실행결과</h3>

![Screenshot from 2024-04-27 22-50-31](https://github.com/its-seon/its-seon.github.io/assets/145862553/1365e7d5-3f89-41d3-b43b-95c315af72c2)

//
//  main.c
//  Astar
//
//  Created by 郭博仁 on 2020/6/10.
//  Copyright © 2020 郭博仁. All rights reserved.
//

/**
     1.将初始节点放入到open列表中。
     2.判断open列表。如果为空，则搜索失败。如果open列表中存在目标节点，则搜索成功。
     3.从open列表中取出F值最小的节点作为当前节点，并将其加入到close列表中。
     4.计算当前节点的相邻的所有可到达节点，生成一组子节点。对于每一个子节点：
        a.如果该节点在close列表中，则丢弃它
        b.如果该节点在open列表中，则检查其通过当前节点计算得到的F值是否更小，如果更小则更新其F值，并将其父节点设置为当前节点。
        c.如果该节点不在open列表中，则将其加入到open列表，并计算F值，设置其父节点为当前节点。
     5.转到2步骤
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>

// 地图宽
#define WID 20
// 地图高
#define HEI 20
// 平地
#define LAND 'o'
// 障碍物
#define OBSTACLE 'H'
// 障碍物生成率
#define OBSTACLE_RATE 0.3
// 起始点
#define START '@'
// 目标点
#define DEST '%'
// 路径
#define ROAD '#'
// 垂直距离
#define VH_DIST 10
// 斜向距离
#define X_DIST 14

// 点结构
typedef struct _Point {
    int x;
    int y;
    int H;
    int G;
    int F;
    char val;
    struct _Point *parent;
    struct _Point *next;
} Point;

// 地图
char map[HEI][WID] = {
    'H', 'H', 'H', 'o', 'o', 'H', 'o', 'o', 'o', 'o',
    'o', 'o', 'o', 'H', 'o', 'o', 'o', 'o', 'o', 'o',
    'H', 'o', 'o', 'o', 'H', 'o', 'o', 'o', 'o', 'H',
    'o', 'H', 'o', 'o', 'o', 'o', 'o', 'o', 'o', 'o',
    'o', 'o', 'o', 'o', 'H', 'o', 'H', 'H', 'o', 'o',
    'o', 'o', 'o', 'o', 'o', 'o', 'H', 'o', 'o', 'o',
    'o', 'o', 'H', 'o', 'H', 'H', 'o', 'o', 'H', 'o',
    'o', 'o', 'o', 'H', 'o', 'H', 'H', 'o', 'o', 'o',
    'o', 'o', 'o', 'o', 'o', 'H', 'o', 'H', 'o', 'o',
    'H', 'H', 'o', 'o', 'o', 'o', 'H', 'o', 'o', 'o'
};

// 起始点，目标点
Point *start, *dest;
// 开放列表
Point *openList;
// 关闭列表
Point *closeList;

// 在内存中分配一个点
Point * newPoint(int, int, char, Point *);
// 从内存中释放一个点
void delPoint(Point *);
// 向List中append一个点
void addList(Point **, Point *);
// 删除List中某个点
void delInList(Point **, Point *);
// 销毁整个List
void freeList(Point *);
// 在List中查找坐标对应的点
Point * findInList(Point *, int, int);
// 输出点信息
void printPointInfo(Point *);
// 打印List
void printList(Point *, char *);
// 初始化随机地图
void initMap(int);
// 打印地图
void printMap(void);
// 生成起始点和目标点
void initStartAndDest(int, int, int, int);
// 计算H值
int calcH(Point *, Point *);
// 计算G值
int calcG(Point *, Point *);
// 计算F值
int calcF(Point *);
// 获取最小F值得点
Point * getMinFPoint(Point **);
// 获取某点周围所有可达点
Point * getAllReachablePoints(Point *);



int main(int argc, const char * argv[]) {
    initMap(1);
    initStartAndDest(-1, -1, -1, -1);
//    initStartAndDest(2,3,6,7);
//    start = newPoint(1, 6, START, NULL);
//    dest = newPoint(4, 2, DEST, NULL);
    printMap();
    calcH(dest, start);
    calcG(start, start);
    calcF(start);
    addList(&openList, start);
    Point *current = NULL;
    Point *findDest = NULL;
    Point *openTmp = NULL;
    Point *newP = NULL;
    while(openList)
    {
//        printList(openList, "openList");
        if ((current = getMinFPoint(&openList)) == NULL)
        {
            printf("get Min F point error\n");
            return -1;
        }
        
        
        delInList(&openList, current);
        addList(&closeList, current);
        Point *tmpList = getAllReachablePoints(current);
//        printList(tmpList, "tmpList");
        for (Point *tmp = tmpList; tmp != NULL; tmp = tmp->next)
        {
            if (findInList(closeList, tmp->x, tmp->y))
            {
                continue;
            }
            else
            {
                if ((openTmp = findInList(openList, tmp->x, tmp->y)) != NULL)
                {

                    int newF = calcG(openTmp, current) + openTmp->G + current->H;
                    int oldF = current->G + current->H;
                    if (newF < oldF)
                        current->parent = openTmp;

                }
                else
                {
                    newP = newPoint(tmp->x, tmp->y, tmp->val, current);
                    addList(&openList, newP);
                }
            }
        }
        freeList(tmpList);
        if ((findDest = findInList(openList, dest->x, dest->y)) != NULL)
        {
            break;
        }
    }
    if (findDest && findDest->parent)
    {
        printf("找到路径\n");
        Point *tmp = findDest->parent;
        while (tmp)
        {
            if (tmp->x != start->x || tmp->y != start->y)
            {
//                printPointInfo(tmp);
                map[tmp->x][tmp->y] = ROAD;
            }
            tmp = tmp->parent;
        }
        printMap();
    }
    else
        printf("未找到路径\n");
	freeList(openList);
	freeList(closeList);
    return 0;
}

Point * getAllReachablePoints(Point *p)
{
    Point *list = NULL;
    Point *cur = NULL;
    Point *tmp = NULL;
    for (int i = p->x - 1; i <= p->x + 1; i++)
    {
        if (i < 0 || i >= HEI)
            continue;
        for (int j = p->y - 1; j <= p->y + 1; j++)
        {
            if (j < 0 || j >= WID)
                continue;
            if (i == p->x && j == p->y)
                continue;
            if (map[i][j] == OBSTACLE)
                continue;
            if (i == p->x - 1 && j == p->y - 1 && map[p->x - 1][p->y] == OBSTACLE && map[p->x][p->y - 1] == OBSTACLE)
                continue;
            if (i == p->x - 1 && j == p->y + 1 && map[p->x - 1][p->y] == OBSTACLE && map[p->x][p->y + 1] == OBSTACLE)
                continue;
            if (i == p->x + 1 && j == p->y - 1 && map[p->x + 1][p->y] == OBSTACLE && map[p->x][p->y - 1] == OBSTACLE)
                continue;
            if (i == p->x + 1 && j == p->y + 1 && map[p->x + 1][p->y] == OBSTACLE && map[p->x][p->y + 1] == OBSTACLE)
                continue;
            tmp = newPoint(i, j, map[i][j], p);
            tmp->H = calcH(dest, tmp);
            tmp->G = calcG(p, tmp);
            tmp->F = calcF(tmp);
            if (list == NULL)
                list = tmp;
            else
                cur->next = tmp;
            cur = tmp;
        }
    }
    return list;
}

Point * getMinFPoint(Point **list)
{
    Point *tmp = *list;
    if (tmp == NULL)
        return NULL;
    int minF = tmp->F;
    Point *ret = tmp;
    while(tmp)
    {
        if (tmp->F < minF)
        {
            minF = tmp->F;
            ret = tmp;
        }
        tmp = tmp->next;
    }
    return ret;
}

int calcH(Point *dest, Point *p)
{
    int dx = abs(dest->x - p->x);
    int dy = abs(dest->y - p->y);
    return (dx + dy) * VH_DIST;
}

int calcG(Point *start, Point *p)
{
    int dx = abs(start->x - p->x);
    int dy = abs(start->y - p->y);
    if (dx == 1 && dy == 1)
        return start->G + X_DIST;
    else
        return start->G + VH_DIST;
}

int calcF(Point *p)
{
    return p->H + p->G;
}

Point * newPoint(int x, int y, char val, Point * p)
{
    Point *newPt = (Point *)calloc(1, sizeof(Point));
    newPt->val = val;
    newPt->x = x;
    newPt->y = y;
    newPt->H = 0;
    newPt->G = 0;
    newPt->F = 0;
    newPt->parent = p;
    newPt->next = NULL;
    return newPt;
}

void delPoint(Point *p)
{
    if (p)
        free(p);
}

void addList(Point **list, Point *p)
{
    if (*list == NULL)
    {
        *list = p;
        return;
    }
    Point *tmp = *list;
    Point *prev = NULL;
    while (tmp)
    {
        if (tmp == p)
            return;
        prev = tmp;
        tmp = tmp->next;
    }
    prev->next = p;
}

void delInList(Point **list, Point *p)
{
    Point *head = *list;
    if (head == p)
    {
        *list = head->next;
        p->next = NULL;
    }
    Point *cur = head->next;
    Point *prev = head;
    while (cur)
    {
        if (cur == p)
        {
            prev->next = cur->next;
            cur->next = NULL;
            return;
        }
        prev = cur;
        cur = cur->next;
    }
    
}

void freeList(Point *list)
{
    Point *tmp = list;
    Point *next = NULL;
    while (tmp)
    {
        next = tmp->next;
        if (tmp)
        {
            delPoint(tmp);
        }
        tmp = next;
    }
}

Point * findInList(Point *list, int x, int y)
{
    Point *tmp = list;
    while (tmp)
    {
        if (tmp->x == x && tmp->y == y)
            return tmp;
        tmp = tmp->next;
    }
    return NULL;
}

void printPointInfo(Point* p)
{
    printf("Point [des:%p, x:%d, y:%d, val:%c, next:%p, parent:%p, g:%d, h:%d, f:%d]\n",
           p, p->x, p->y, p->val, p->next, p->parent, p->G, p->H, p->F);
}

void printList(Point *list, char *listName)
{
    printf("############### Print List %s ###############\n", listName == NULL ? "" : listName);
    Point *tmp = list;
    while(tmp)
    {
        printPointInfo(tmp);
        tmp = tmp->next;
    }
    puts("");
}

void initMap(int random)
{
    if (!random)
        return;
    srand((unsigned int)time(NULL));
    for (int i = 0; i < HEI; i++)
    {
        for (int j = 0; j < WID; j++)
        {
            if ((double)rand() / RAND_MAX < OBSTACLE_RATE)
                map[i][j] = OBSTACLE;
            else
                map[i][j] = LAND;
        }
    }
}

void printMap(void)
{
    printf("############### Print Map [可走的路:%c, 障碍物:%c, 起点:%c, 终点:%c, 路径:%c] ###############\n",
           LAND, OBSTACLE, START, DEST, ROAD);
    for (int i = 0; i < HEI; i++)
    {
        for (int j = 0; j < WID; j++)
        {
            if (map[i][j] == ROAD)
                printf("\033[1;35m%3c\033[0m", map[i][j]);
            else if (map[i][j] == START)
                printf("\033[1;31m%3c\033[0m", map[i][j]);
            else if (map[i][j] == DEST)
                printf("\033[1;34m%3c\033[0m", map[i][j]);
            else if (map[i][j] == OBSTACLE)
                printf("\033[1;32m%3c\033[0m", map[i][j]);
            else
                printf("\033[1;30m%3c\033[0m", map[i][j]);
//            if (map[i][j] == ROAD)
//                printf("%3c", map[i][j]);
//            else if (map[i][j] == START)
//                printf("%3c", map[i][j]);
//            else if (map[i][j] == DEST)
//                printf("%3c", map[i][j]);
//            else if (map[i][j] == OBSTACLE)
//                printf("%3c", map[i][j]);
//            else
//                printf("%3c", map[i][j]);
        }
        puts("");
    }
    puts("");
}

void initStartAndDest(int startX, int startY, int destX, int destY)
{
    if (startX != -1 && startY != -1 && destX != -1 && destY != -1)
    {
        map[startX][startY] = START;
        start = newPoint(startX, startY, START, NULL);
        map[destX][destY] = DEST;
        dest = newPoint(destX, destY, DEST, NULL);
    }
    else
    {
        srand((unsigned int)time(NULL));
        int x = -1, y = -1;
        while (1)
        {
            x = rand() % WID;
            y = rand() % HEI;
            if (map[x][y] == LAND)
            {
                map[x][y] = START;
                start = newPoint(x, y, START, NULL);
                break;
            }
        }
        while (1)
        {
            x = rand() % WID;
            y = rand() % HEI;
            if (map[x][y] == LAND)
            {
                map[x][y] = DEST;
                dest = newPoint(x, y, DEST, NULL);
                break;
            }
        }
    }
    start->H = calcH(dest, start);
    start->G = 0;
    start->F = calcF(start);
    dest->H = 0;
    dest->G = 0;
    dest->F = 0;
}

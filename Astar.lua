--[[
    Author:guoboren

    算法介绍：
    G = [到起始点的距离]
    H = [到目标点的距离]
    F = G + H
    A*寻路算法是一种启发式算法，根据路径上每个点 F 最小来筛选出最短路径
    其中每个点 H 是用曼哈顿距离计算
    而每个点 G 则是根据当前点的父节点到起点的距离加上当前点到父节点的距离（其中斜向为14（根号2）， 垂直/水平方向为10）

    两个链表：
    openList:可以走的点
    closeList:已检查过的点

    算法流程：
    1、计算起始点的G、H、F，并加入openList
    2、进行循环判断
        a、openList是否为空，如果为空则跳出循环，路径未找到
        b、从openList中找出 F 最小的点（这里称为路径点）并从openList中删除，再添加到closeList表明该点已经判断过了，找出该点可达的所有点（每个点这里称为当前点）
            对于当前点：
                i、如果在closeList中，不需要管
                ii、如果在openList中，说明之前可以走这个点，但是因为当时的 F 值并不是最小的，故而选择其他路。所以需要判断经过当前点到路径点的 F 值
                    和路径点的 F 值，如果小于路径点的F值，则说明经过当前点到路径点距离比不经过当前点到路径点的距离更短，那么就把当前点的父节点设为路径点，
                    否则不需要管，在后续的循环中迟早会取出当前点作为路径点（因为排除了前一情况，）
                iii、如果不在openList中，那么计算当前点的G、H、F，并设置父节点为路径点，加入openList
        c、判断目标点是否已经在openList中，如果不在，继续重复第2步的循环，否则跳出循环，表明路径已找到

]]
-- 点结构 {x,y,val,parent,h,g,f}

local WID = 10
local HEI = 10
local X_DIS = 14
local VH_DIS = 10
local ROAD = ' ~ '
local OBSTACLE = ' % '
local START = '$$$'
local DEST = '###'
local OBS_RATE = 0.2
local PATH = '[*]'

local map = {

}
local openList = {}
local closeList = {}
local startPoint = nil
local destPoint = nil

function getPointInfo(p)
    return string.format("[INFO] [x:%d y:%d val:%s G:%d H:%d F:%d self:%s parent:%s]", p.x, p.y, p.val, p.g, p.h, p.f, p, p.parent)
end

function printPoints(list)
    for k, v in pairs(list) do
        print(string.format("[%d] %s", k, getPointInfo(v)))
    end
end

function init()
    math.randomseed(os.time())
    for i = 1, HEI do
        map[i] = {}
        for j = 1, WID do
            if math.random() <= OBS_RATE then
                map[i][j] = OBSTACLE
            else
                map[i][j] = ROAD
            end
        end
    end
    local x, y
    while true do
        x = math.random(1, HEI)
        y = math.random(1, WID)
        if map[x][y] == ROAD then
            startPoint = {x = x, y = y, val = START, parent = nil, h = 0, g = 0, f = 0}
            map[x][y] = START
            break
        end
    end
    while true do
        x = math.random(1, HEI)
        y = math.random(1, WID)
        if map[x][y] == ROAD then
            destPoint = {x = x, y = y, val = DEST, parent = nil, h = 0, g = 0, f = 0}
            map[x][y] = DEST
            break
        end
    end
    startPoint.h = getPointH(destPoint, startPoint)
    startPoint.f = getPointF(startPoint)
    destPoint.g = getPointG(startPoint, destPoint)
    destPoint.f = getPointF(startPoint)
end

function printMap()
    local title = {}
    for i = 0, WID do
        local str
        if i >= 10 then
            str = string.format("%d ", i)
        else
            str = string.format("0%d ", i)
        end
        table.insert(title, str)
    end
    print(table.concat(title, '  '))
    for i = 1, HEI do
        if i >= 10 then
            print(string.format("%d   %s", i, table.concat(map[i], '  ')))
        else
            print(string.format("0%d   %s", i, table.concat(map[i], '  ')))
        end
        
    end
    print()
end

function getMinFPoint()
    local index = 1
    local minPoint = openList[1]
    assert(type(minPoint.f) == 'number')
    for k, point in ipairs(openList) do
        if point.f < minPoint.f then
            minPoint = point
            index = k
        end
    end
    table.remove(openList, index)
    table.insert(closeList, minPoint)
    return minPoint
end

function findPointInList(list, p)
    for k, v in pairs(list) do
        if v.x == p.x and v.y == p.y then
            return true
        end
    end
    return false
end

function getPointF(p)
    return p.h + p.g
end

function getPointG(start, p)
    local tmpG
    if math.abs(start.x - p.x) == 1 and math.abs(start.y - p.y) == 1 then
        tmpG = X_DIS
    else
        tmpG = VH_DIS
    end
    return start.g + tmpG
end

function getPointH(dest, p)
    return math.abs(dest.x - p.x) + math.abs(dest.y - p.y)
end

function getSurrendReachablePoints(p)
    local list = {}
    for i = p.x - 1, p.x + 1 do
        for j = p.y - 1, p.y + 1 do
            local flag = true
            if i < 1 or i > HEI or j < 1 or j > WID  then
                flag = false
            else
                if map[i][j] == OBSTACLE then -- 是否是障碍物
                    flag = false
                end
                if i == p.x and j == p.y then -- 是否是本点
                    flag = false
                end
                if (i - p.x == -1 and j - p.y == -1) or (i - p.x == 1 and j - p.y == -1) then -- 上方对角是否可达
                    if map[i][p.y] == OBSTACLE and map[p.x][p.y - 1] == OBSTACLE then
                        flag = false
                    end
                end
                if (i - p.x == -1 and j - p.y == 1) or (i - p.x == 1 and j - p.y == 1) then -- 下方对角是否可达
                    if map[i][p.y] == OBSTACLE and map[p.x][p.y + 1] == OBSTACLE then
                        flag = false
                    end
                end
            end 
            if flag then
                local point = {x = i, y = j, val = ROAD, g = 0, h = 0, f = 0, parent = p}
                point.h = getPointH(destPoint, point)
                point.g = getPointG(p, point)
                point.f = getPointF(point)
                table.insert(list, point)
            end
        end
    end
    return list
end

function findInList(list, p)
    for k, v in pairs(list) do
        if v.x == p.x and v.y == p.y then
            return k
        end
    end
    return false
end

function updateOLPointInfo(minFPoint, reachablePoint)
    local newF = getPointG(minFPoint, reachablePoint) + reachablePoint.h
    local oldF = reachablePoint.f
    if newF < oldF then
        openList[index].g = getPointG(minFPoint, reachablePoint)
        openList[index].f = newF
        openList[index].parent = minFPoint
    end
end

function insertOrUpdateOpenList(minFPoint, reachableList)
    for k, reachablePoint in pairs(reachableList) do
        if not findInList(closeList, reachablePoint) then
            local index = findInList(openList, reachablePoint)
            if index then
                updateOLPointInfo(minFPoint, reachablePoint)
            else
                table.insert(openList, reachablePoint)
            end
        end
    end
end

function findPath()
    table.insert(openList, startPoint)
    while #openList > 0 do
        local minFPoint = getMinFPoint()
        local reachableList = getSurrendReachablePoints(minFPoint)
        insertOrUpdateOpenList(minFPoint, reachableList)
        if findInList(openList, destPoint) then
            break
        end
    end
end

function printResult()
    local findDest = openList[findInList(openList, destPoint)]
    if findDest then
        print("find path\n")
        local tmp = findDest.parent
        while tmp.x ~= startPoint.x or tmp.y ~= startPoint.y  do
            map[tmp.x][tmp.y] = PATH
            tmp = tmp.parent
        end
        printMap()
    else
        print("not find path")
    end
end

-- main --
function start()
    print(string.format("起始点:%s 终点:%s 路径:%s 障碍物:%s 可走的路:%s", START, DEST, PATH, OBSTACLE, ROAD))
    init()
    printMap()
    findPath()
    printResult()
end

start()

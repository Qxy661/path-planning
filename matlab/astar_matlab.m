%% A* 路径规划 - MATLAB 实现（对比 Python 手写版）
% 科研验证用途：同一算法，Python手写 / MATLAB / Nav2 三种实现对比
%
% 对应：docs/06-MATLAB仿真vs实际部署.md
% 依赖：MATLAB Navigation Toolbox (plannerAStarGrid)
%
% 说明：MATLAB 在科研中作为算法验证/原型工具，Nav2 作为部署标准。
% 这个脚本展示两种方式：
%   1. 手写 A*（理解原理）
%   2. Navigation Toolbox plannerAStarGrid（官方库）

function astar_matlab()
    %% 创建栅格地图
    grid = zeros(20, 20);
    grid(5:15, 7) = 1;      % 竖墙
    grid(10, 3:7) = 1;      % 横墙
    grid(2:18, 13) = 1;     % 第二道竖墙
    grid(15, 13:18) = 1;    % 底部墙

    % 转占用地图（0=空地→false, 1=障碍→true）
    map = binaryOccupancyMap(grid == 1, 1);  % 障碍=1

    start = [2, 2];
    goal = [18, 18];

    %% 方式1：手写 A*（理解原理）
    disp('=== 手写 A* ===');
    [path_manual, visited] = astar_manual(grid, [1,1], [18,18]);
    disp(['访问节点数: ', num2str(visited)]);
    disp(['路径长度: ', num2str(size(path_manual,1))]);

    %% 方式2：Navigation Toolbox 官方库
    disp('=== Navigation Toolbox plannerAStarGrid ===');
    planner = plannerAStarGrid(map);
    path_toolbox = plan(planner, start, goal);

    %% 可视化对比
    figure;
    subplot(1,2,1);
    show(map);
    hold on;
    plot(start(1), start(2), 'go', 'MarkerSize', 12);
    plot(goal(1), goal(2), 'ro', 'MarkerSize', 12);
    title('Navigation Toolbox A*');

    subplot(1,2,2);
    imagesc(grid);
    colormap(gray);
    hold on;
    if ~isempty(path_manual)
        plot(path_manual(:,2), path_manual(:,1), 'r-', 'LineWidth', 2);
    end
    plot(1, 1, 'go', 'MarkerSize', 12);
    plot(18, 18, 'ro', 'MarkerSize', 12);
    title('手写 A*');

    saveas(gcf, 'C:\Users\qixiaoyang\vla-roadmap\path-planning\results\astar_comparison.png');
    disp('对比图已保存: results/astar_comparison.png');
end

%% 手写 A*（MATLAB 版，与 Python 版逻辑一致）
function [path, visited] = astar_manual(grid, start, goal)
    rows = size(grid, 1);
    cols = size(grid, 2);

    openList = struct('pos', {}, 'f', {}, 'g', {});
    cameFrom = containers.Map();
    gScore = containers.Map();

    startKey = mat2str(start);
    goalKey = mat2str(goal);
    gScore(startKey) = 0;
    openList(end+1) = struct('pos', start, 'f', heuristic(start, goal), 'g', 0);
    visited = 0;

    while ~isempty(openList)
        % 找 f 最小
        [~, idx] = min([openList.f]);
        current = openList(idx);
        openList(idx) = [];
        visited = visited + 1;

        currentKey = mat2str(current.pos);
        if isequal(current.pos, goal)
            % 回溯路径
            path = reconstructPath(cameFrom, currentKey, startKey);
            return;
        end

        % 8邻域
        for dr = -1:1
            for dc = -1:1
                if dr == 0 && dc == 0
                    continue;
                end
                nr = current.pos(1) + dr;
                nc = current.pos(2) + dc;
                if nr < 1 || nr > rows || nc < 1 || nc > cols
                    continue;
                end
                if grid(nr, nc) == 1
                    continue;
                end
                % 对角穿墙检查
                if abs(dr) == 1 && abs(dc) == 1
                    if grid(current.pos(1)+dr, current.pos(2)) == 1 || ...
                       grid(current.pos(1), current.pos(2)+dc) == 1
                        continue;
                    end
                end

                stepCost = 1.0;
                if abs(dr) + abs(dc) == 2
                    stepCost = 1.414;
                end
                neighbor = [nr, nc];
                neighborKey = mat2str(neighbor);
                tentativeG = gScore(currentKey) + stepCost;

                if ~isKey(gScore, neighborKey) || tentativeG < gScore(neighborKey)
                    cameFrom(neighborKey) = currentKey;
                    gScore(neighborKey) = tentativeG;
                    f = tentativeG + heuristic(neighbor, goal);
                    openList(end+1) = struct('pos', neighbor, 'f', f, 'g', tentativeG);
                end
            end
        end
    end
    path = [];
    visited = visited;
end

function h = heuristic(a, b)
    h = norm(a - b);  % 欧氏距离
end

function path = reconstructPath(cameFrom, currentKey, startKey)
    path = [];
    key = currentKey;
    while ~strcmp(key, startKey)
        pos = eval(key);
        path = [pos; path];
        key = cameFrom(key);
    end
    path = [eval(startKey); path];
end

function [desired_state] = student_control_nav(t, qd, map, goal)
  
persistent t_init;

if nargin > 1
    start = reshape(qd{1}.pos, 1,3);
    path = dijkstra(map, start, reshape(goal,1,3), false);
    if(isempty(path))
        error('No path made');
    end
    trajectory_generator([], [],map, path);
    t_init = 0;
    return
end

if t_init == 0
    t_init = t;
end

qn = 1;
desired_state = trajectory_generator(t-t_init, qn);

end

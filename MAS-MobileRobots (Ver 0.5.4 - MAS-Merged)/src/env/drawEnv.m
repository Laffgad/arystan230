function drawEnv(Env)
    % Env.bounds = [xmin xmax; ymin ymax]
    xmin = Env.bounds(1,1); xmax = Env.bounds(1,2);
    ymin = Env.bounds(2,1); ymax = Env.bounds(2,2);

    % Plot each obstacle polyshape
    for i = 1:numel(Env.obstacles)
        o = Env.obstacles(i);

        plot(o.poly, ...
            'FaceColor', o.faceColor, ...
            'FaceAlpha', o.faceAlpha, ...
            'EdgeColor', o.edgeColor, ...
            'LineWidth', o.edgeWidth);
    end

    % Axes settings
    xlim([xmin xmax]);
    ylim([ymin ymax]);
    grid on;
    box on;
end
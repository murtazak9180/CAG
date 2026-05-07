function [] = display_init_connectivity(posi, veli, params)
x = posi(1,:); y = posi(2,:);
vx = veli(1,:); vy = veli(2,:);

displayInitState(x, y, vx, vy, 1, [1,0,0])

    for i = 1:params.n
        for j = i+1:params.n
            edges(i,j) = plot([x(i), x(j)], [y(i), y(j)], 'Color', [0 0 0], 'LineWidth', 1.0, 'LineStyle', '-');
            if norm([x(i); y(i)] - [x(j); y(j)]) > params.rs
                set(edges(i,j), 'Visible', 'off');
            else
                set(edges(i,j), 'Visible', 'on');
            end
        end
    end
end


function draw_Run(state, stance_point, swing_position, leg_state)
    x_min = -0.3;
    x_max = 0.3;
    y_min = -0.6;
    y_max = 0.2;
    % leg_state: left leg state
    pcx = state(1,:);
    pcy = state(2,:);
    sita = state(3,:);
    for i = 1:1:length(leg_state)
        pA = [pcx(i); pcy(i)];
        hold off
        draw_robot(sita(i), pcx(i), pcy(i));
        hold on
        stance_foot = stance_point(i,:);
        swing_foot = swing_position(i,:);
        if leg_state(i) == 0 % left swing
            % plot left leg
            pB = IK(pA, swing_foot);
            plot([pcx(i), pB(1)], [pcy(i), pB(2)], 'LineWidth',2, 'color','r');
            hold on
            plot([pB(1), swing_foot(1)], [pB(2), swing_foot(2)], 'LineWidth',2, 'color','r');
            hold on
            % plot right leg
            pB = IK(pA, stance_foot);
            plot([pcx(i), pB(1)], [pcy(i), pB(2)], 'LineWidth',2, 'color','b');
            hold on
            plot([pB(1), stance_foot(1)], [pB(2), stance_foot(2)], 'LineWidth',2, 'color','b');
            hold on
        else % left stance
            % plot left leg
            pB = IK(pA, stance_foot);
            plot([pcx(i), pB(1)], [pcy(i), pB(2)], 'LineWidth',2, 'color','r');
            hold on
            plot([pB(1), stance_foot(1)], [pB(2), stance_foot(2)], 'LineWidth',2, 'color','r');
            hold on
            % plot right leg
            pB = IK(pA, swing_foot);
            plot([pcx(i), pB(1)], [pcy(i), pB(2)], 'LineWidth',2, 'color','b');
            hold on
            plot([pB(1), swing_foot(1)], [pB(2), swing_foot(2)], 'LineWidth',2, 'color','b');
            hold on            
        end
        axis equal
        xlim([x_min + pcx(i), x_max + pcx(i)]);
        ylim([y_min + pcy(i), y_max + pcy(i)]);   
        % plot ground
        plot([x_min + pcx(i), x_max + pcx(i)], [0, 0], 'LineWidth',2, 'color','k');
        drawnow;
        pause(0.03);
    end
    
end
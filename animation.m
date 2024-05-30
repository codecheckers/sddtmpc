% figure (1)
% 
% for i=2:60
% 
%     leader_state = state_ref_history(i,:);
%     plot_robot(leader_state+[p*cos(leader_state(3)),p*sin(leader_state(3)),0],p,'red');
%     hold on
%     ref_state = state_desired_history(i,:);
%     plot_robot(ref_state,p,'green')
%     state_robot = state_history(i,:);
%     plot_robot(state_robot,p,'blue')
%     state_robot_SDDTMPC = state_history_SDDTMPC(i,:);
%     plot_robot(state_robot_SDDTMPC,p,'black')
%     xlim([-0.1,0.4])
%     ylim([-0.3,0.3])
%     pause(0.2)
%     hold off
% end

f=open('stage_2_SDDTMPC.fig');
h = get(gca, 'Children');

legend('','','','nominal trajectory (robot pulled down)','','real trajectory (robot pulled down)',...
    'nominal trajectory (robot pulled up)','','real trajectory (robot pulled up)','','', 'nominal/real trajectory (no disturbance)')
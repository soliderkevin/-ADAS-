%% Timer countdown testing.
%% How are they going to change the value if it's stucking in this while? Distance part.
time_limit = 4;

times_up  = 0;
Statement = 0;
loop_start = tic;
while ~times_up
    
    dt = 0.25*rand;
    pause(dt);
    time_elapsed = toc(loop_start);
    time_remaining = time_limit - time_elapsed;
    times_up = time_remaining <=0;

 
    if ~times_up
        status_msg = 'Continuing following countdown';
    else
        Change_index = 1;
        status_msg =  'Ready to switch lane';
    end

    disp(['Waited',num2str(dt,'%5.4f') 's ; time left: ', ...
        num2str(time_remaining, '%6.3f'),'s. ',status_msg])

     if Statement>2
            break;
     end
   Statement =  Statement+1;
end



function clear_sim_state()
% CLEAR_SIM_STATE  Wipe sim variables without touching TUNER_GAINS or figures.
    evalin('base', ...
        ['clearvars -except TUNER_GAINS; ' ...
         'clear functions;']);          % flushes persistent vars in controllers
end
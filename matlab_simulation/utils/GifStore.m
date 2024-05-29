%% Store a Gif
% file: GifStore.m
% author: Federico Oliva 
% date: 20/02/2024
% description: call a plot and store a gif
function GifStore(f,G,delay)

    % wait and see
    pause(delay)
    drawnow

    % gif
    exportgraphics(f,G,'Append',true);

end

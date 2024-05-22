%% MonteCarlo Test
% file: MonteCarloTest.m
% author: Federico Oliva 
% date: 21/04/2024
% description: run a simulation N times and gets performance metrics
function out = MonteCarloTest(Niter,i)

    % set rng
    rng(i);
    out.posSucc = [];
    out.posFail = [];

    % iterate
    for ii=1:Niter           

        % run analysis
        try
        testLocalization;
        catch
            a=1;% I don't know
        end

        % get coverage
        out.C0(ii,1) = computeCoverage(agents_pos,1e4,3,0);
        out.C1(ii,1) = computeCoverage(agentsEst(:,3:4),1e4,3,0);

        % save pos
        out.PosInit(ii).val = agents_pos;
        out.PosTrue(ii).val = agentsEst(:,1:2);
        out.PosEst(ii).val = agentsEst(:,3:4);

        % save iter
        out.Iter(ii,1) = iter;

        % get localization precision
        tmp = norm(vecnorm(agentsEstDiff,2,2));
        if isnan(tmp)
            tmp = -1;
            out.posFail = [out.posFail ii];
        else
            out.posSucc = [out.posSucc ii];
        end
        out.ACC(ii,1) = tmp;        

        % show
        clc
        disp(['iteration: ', num2str(ii)]);
        disp(['C0: ', num2str(mean(out.C0(out.posSucc)))]);
        disp(['C1: ', num2str(mean(out.C1(out.posSucc)))]);
        disp(['ACC: ', num2str(mean(out.ACC(out.posSucc)))]);
        disp(['ITER: ', num2str(mean(out.Iter(out.posSucc)))]);
        disp(['SUCC: ', num2str(numel(out.posSucc)/ii)]);

    end

    out.SUCC = num2str(numel(out.posSucc)/Niter);

end
clear; clc; close all;
addpath(genpath(pwd));

P   = simParameters();
Env = buildEnvManual();
%load("Arystan228.mat");
CS = buildCSpaceGridXY(Env,P);

inflationRadius = P.robotRadius *1.5;
spacingFactor   = 1.0;

[~, circlesEB]  = approxEnvWithCirclesMulti(Env, inflationRadius, spacingFactor);
[~, circlesRep] = approxEnvWithCirclesMulti(Env, 0.0,            spacingFactor);
[envXY, envR]   = packCirclesForRepulsion(circlesRep);

nR = P.nRobots;

pathXY = cell(nR,1);
pathEB = cell(nR,1);
Xref   = cell(nR,1);
Uref   = cell(nR,1);

figure(1); clf; set(gcf,'Color','w'); hold on; axis equal;
drawEnv(Env);
title('Planned paths'); xlabel('x'); ylabel('y');
cols = lines(nR);

% --- Initial Path Planning (Offline) ---
for i = 1:nR
    x0 = P.x0s(i,:).';
    xg = P.xgs(i,:).';

    pathXY{i} = astarGridXY(CS, x0, xg, 8);
    if isempty(pathXY{i}), error('A* failed for robot %d.', i); end

    [pathEB{i}, infoEB] = elasticBandSmooth(pathXY{i}, circlesEB, P.EB, CS);
    disp(struct('robot',i,'EB_iters',infoEB.iters,'EB_ok',infoEB.ok,'EB_meanMove',infoEB.meanMove));

    [Xref{i}, Uref{i}] = pathToUnicycleRef(pathEB{i}, P.dt, P.vMax, P.wMax, xg(3), P.ref);
    if isempty(Xref{i}) || isempty(Uref{i})
        error('Reference is empty for robot %d. Check pathToUnicycleRef output.', i);
    end

    plot(pathXY{i}(:,1), pathXY{i}(:,2), '-',  'Color', cols(i,:), 'LineWidth', 1.0);
    plot(pathEB{i}(:,1), pathEB{i}(:,2), '--', 'Color', cols(i,:), 'LineWidth', 2.0);
    plot(x0(1), x0(2), 'o', 'Color', cols(i,:), 'MarkerFaceColor', cols(i,:), 'MarkerSize', 6);
    plot(xg(1), xg(2), 's', 'Color', cols(i,:), 'MarkerFaceColor', cols(i,:), 'MarkerSize', 6);
end

Xarr  = zeros(P.k+1, 3, nR);
Uarr  = zeros(P.k,   2, nR);
Earr  = zeros(P.k,   3, nR);

kHat  = repmat(P.adapt.k0, 1, nR);
kHarr = zeros(P.k+1, 3, nR);

for i = 1:nR
    Xarr(1,:,i)  = P.x0s(i,:);
    kHarr(1,:,i) = kHat(:,i).';
end

holdCount = zeros(nR,1);
ids = (1:nR).';

leaderLocked  = false;
leaderIdxLock = NaN;

leaderIdxHist = nan(P.k,1);
formOnHist    = false(P.k,1);
degTrigHist   = zeros(P.k,nR);

XrCmdHist = nan(P.k,3,nR);
UrCmdHist = nan(P.k,2,nR);

RcmdHist     = nan(P.k,1);
w0Hist       = nan(P.k,1);
vLeadCmdHist = nan(P.k,1);
dObsMinHist  = nan(P.k,1);
dClrLeadHist = nan(P.k,1);

% --- Initialize Cost History ---
costHist = zeros(P.k, 1);

% --- Track Participation & Status ---
hasJoined      = false(nR, 1); 
currentMembers = false(nR, 1); 
isFinished     = false(nR, 1);

formState = struct();
formState.phiRef  = NaN(nR,1);
formState.thdPrev = P.x0s(:,3);

formHL = struct();
formHL.Rprev = P.form.R;

leaderArcState = cell(nR,1);

% --- ONLINE LOGIC STATE ---
formationDisabled = false;
replanStartStep   = ones(nR, 1); % Default start index is 1

for k = 1:P.k
    posAll = squeeze(Xarr(k,1:2,:));
    thAll  = squeeze(Xarr(k,3,:));

    % --- Logic to prevent re-entering ---
    validAgents = (~hasJoined | currentMembers) & ~isFinished;
    ignoreMask  = ~validAgents;
    
    % --- Select Leader Only if Formation is NOT Permanently Disabled ---
    if formationDisabled
        leaderIdx = NaN;
        haveLeader = false;
        leaderOutCand = struct('degTrig', zeros(nR,1), 'leaderIdx', NaN, 'commAdj', zeros(nR,nR));
    else
        leaderOutCand = selectLeaderByConnections(posAll.', P.commR, P.dTrig, ids, ignoreMask);
        candHasLeader = any(leaderOutCand.degTrig > 0) && isfinite(leaderOutCand.leaderIdx);

        if leaderLocked
            leaderIdx = leaderIdxLock;
        else
            if candHasLeader
                leaderIdx = leaderOutCand.leaderIdx;
                leaderLocked  = true;
                leaderIdxLock = leaderIdx;
            else
                leaderIdx = NaN;
            end
        end
        haveLeader = isfinite(leaderIdx);
    end

    % --- Calculate Cost of Formation J(t) ---
    if haveLeader
        neighbors = leaderOutCand.commAdj(leaderIdx, :);
        N_active  = sum(neighbors) + 1;

        currentMembers(:) = false;
        currentMembers(leaderIdx) = true;
        currentMembers = currentMembers | neighbors.';
        
        hasJoined = hasJoined | currentMembers;

        startPos  = P.x0s(leaderIdx, 1:2).';
        goalPos   = P.xgs(leaderIdx, 1:2).';
        currPos   = posAll(:, leaderIdx);

        totalDist = hypot(goalPos(1) - startPos(1), goalPos(2) - startPos(2));
        remDist   = hypot(goalPos(1) - currPos(1),  goalPos(2) - currPos(2));
        
        progress  = 1 - (remDist / totalDist);
        progress  = max(0, min(1, progress));

        J = N_active * P.cost.C_base * exp(P.cost.alpha * progress);
        costHist(k) = J;
    else
        currentMembers(:) = false;
        costHist(k) = 0; 
    end
    
    % --- ONLINE LOGIC: Check Cost Threshold & Replan ---
    if costHist(k) >= 180 && ~formationDisabled
        formationDisabled = true;
        disp(['[Step ' num2str(k) '] Cost J=' num2str(costHist(k)) ' >= 180. Formation DESTROYED. Online Replanning...']);
        
        % 1. Immediately drop leader/formation status
        haveLeader = false;
        leaderLocked = false;
        leaderIdxLock = NaN;
        currentMembers(:) = false;
        
        % 2. Replan for EVERY agent independently
        for i = 1:nR
            % Get current state
            pNowState = squeeze(Xarr(k,:,i)).'; % [x; y; theta]
            pNowPos   = pNowState(1:2);
            pNowTh    = pNowState(3);
            
            % Goal
            xg = P.xgs(i,:).';
            
            % Reset Adaptive Gains
            kHat(:,i) = P.adapt.k0;

            % --- PREPARE DYNAMIC OBSTACLE MAP ---
            % We create a temporary map (CS_temp) that includes OTHER robots 
            % as static obstacles so A* avoids them.
            CS_temp = CS;
            circlesEB_temp = circlesEB;
            
            [H_grid, W_grid] = size(CS.occ);
            
            for j = 1:nR
                if i == j, continue; end
                
                pj = posAll(:,j);
                
                % -- Update Grid for A* (Rasterize Neighbor) --
                % 2*Radius to account for C-Space (Agent i radius + Agent j radius)
                distBlock = 2.0 * P.robotRadius; 
                
                % Find bounding box indices
                xMin = floor((pj(1) - distBlock - CS.xs(1))/CS.dx) + 1;
                xMax = ceil( (pj(1) + distBlock - CS.xs(1))/CS.dx) + 1;
                yMin = floor((pj(2) - distBlock - CS.ys(1))/CS.dx) + 1;
                yMax = ceil( (pj(2) + distBlock - CS.ys(1))/CS.dx) + 1;
                
                % Clamp to grid
                xMin = max(1, min(W_grid, xMin));
                xMax = max(1, min(W_grid, xMax));
                yMin = max(1, min(H_grid, yMin));
                yMax = max(1, min(H_grid, yMax));
                
                if xMax >= xMin && yMax >= yMin
                    [Gx, Gy] = meshgrid(xMin:xMax, yMin:yMax);
                    Wx = CS.xs(Gx);
                    Wy = CS.ys(Gy);
                    
                    % Circle check
                    maskObs = (Wx - pj(1)).^2 + (Wy - pj(2)).^2 <= distBlock^2;
                    
                    % Update temporary occupancy
                    linInd = sub2ind([H_grid, W_grid], Gy(maskObs), Gx(maskObs));
                    CS_temp.occ(linInd) = true;
                end
                
                % -- Update Circles for Elastic Band Smoothing --
                % FIXED: Append struct using indexing to avoid dimension mismatch
                obsCirc = struct('c', [pj(1), pj(2)], 'r', P.robotRadius, 'srcIdx', 0);
                circlesEB_temp(end+1) = obsCirc; 
            end
            
            % Explicitly clear the robot's own start cell to prevent "Start is in obstacle" error
            % (This can happen if robots are very close when formation breaks)
            si = round((pNowPos(2) - CS.ys(1))/CS.dx) + 1;
            sj = round((pNowPos(1) - CS.xs(1))/CS.dx) + 1;
            if si>=1 && si<=H_grid && sj>=1 && sj<=W_grid
                CS_temp.occ(si, sj) = false;
            end

            % --- A* Replanning with Dynamic Obstacles ---
            pathNew = astarGridXY(CS_temp, pNowState, xg, 8);
            
            if isempty(pathNew)
                warning('Robot %d blocked or A* failed. Holding pos.', i);
                Xref{i} = repmat(pNowState.', 2, 1);
                Uref{i} = zeros(2,2);
            else
                % PREPEND START to avoid position jumps
                if norm(pathNew(1,:) - pNowPos.') > 0.01
                    pathNew = [pNowPos.'; pathNew];
                end
                
                % Smooth path (Using updated circlesEB_temp to push away from other robots)
                [pathEB_New, ~] = elasticBandSmooth(pathNew, circlesEB_temp, P.EB, CS_temp);
                pathEB{i} = pathEB_New; 
                
                % Generate Trajectory with INITIAL SPIN
                opts = P.ref;
                opts.startTheta = pNowTh; % Pass current heading to force alignment
                
                [XrefNew, UrefNew] = pathToUnicycleRef(pathEB_New, P.dt, P.vMax, P.wMax, xg(3), opts);
                
                if ~isempty(XrefNew)
                    % Safety overwrite to ensure exact numeric match at step 0
                    XrefNew(1, 1:3) = [pNowPos.', pNowTh];
                    
                    Xref{i} = XrefNew;
                    Uref{i} = UrefNew;
                else
                    warning('Trajectory gen failed for Robot %d.', i);
                end
            end
            
            % 3. Update time index
            replanStartStep(i) = k;
        end
    end

    leaderIdxHist(k) = leaderIdx;
    formOnHist(k)    = haveLeader;
    degTrigHist(k,:) = leaderOutCand.degTrig.';

    XrPlan = zeros(nR,3);
    UrPlan = zeros(nR,2);
    for i = 1:nR
        % --- Indexing Logic to support Replanning ---
        idx = k - replanStartStep(i) + 1;
        
        Nref = size(Xref{i},1);
        kk   = max(1, min(idx, Nref)); 
        
        XrPlan(i,:) = Xref{i}(kk,:);
        UrPlan(i,:) = Uref{i}(kk,:);
    end

    if haveLeader
        leaderPos = posAll(:,leaderIdx);
        vPlan     = UrPlan(leaderIdx,1);

        [cmdHL, formHL] = formationHighLevelStep(leaderPos, vPlan, envXY, envR, formHL, P);
        cmdHL.vLeader = clamp(cmdHL.vLeader, 0, P.form.vLeadMax);

        P.form.R  = cmdHL.R;
        if isfield(cmdHL,'Rdot') && ~isempty(cmdHL.Rdot)
            P.form.Rdot = cmdHL.Rdot;
        end
        P.form.w0 = cmdHL.w0;
        P.form.vLeader = cmdHL.vLeader;

        RcmdHist(k)     = cmdHL.R;
        w0Hist(k)       = cmdHL.w0;
        vLeadCmdHist(k) = cmdHL.vLeader;
        dObsMinHist(k)  = cmdHL.dObsMin;
        dClrLeadHist(k) = cmdHL.dClearLeader;

        [xrL, yrL, thL, vrL, wrL, leaderArcState{leaderIdx}] = leaderRefArcStep( ...
            pathEB{leaderIdx}, leaderPos, cmdHL.vLeader, leaderArcState{leaderIdx}, P);

        XrPlan(leaderIdx,:) = [xrL, yrL, thL];
        UrPlan(leaderIdx,:) = [vrL, wrL];

        [XrCmd, UrCmd, formState] = formationTrajStep( ...
            posAll, thAll, leaderIdx, XrPlan, UrPlan, envXY, envR, formState, P);
    else
        XrCmd = XrPlan;
        UrCmd = UrPlan;
        formState.phiRef(:) = NaN;
        formState.thdPrev   = thAll;
    end

    XrCmdHist(k,:,:) = permute(XrCmd, [3 2 1]);
    UrCmdHist(k,:,:) = permute(UrCmd, [3 2 1]);

    vAll = zeros(nR,1);
    wAll = zeros(nR,1);

    for i = 1:nR
        x  = squeeze(Xarr(k,:,i)).';
        xi = x(1); yi = x(2);

        % --- CHECK GOAL REACHED ---
        distGoal = hypot(xi - P.xgs(i,1), yi - P.xgs(i,2));
        if distGoal < P.track.posTol
            isFinished(i) = true;
        end

        if isFinished(i)
            v = 0; 
            w = 0;
            e = [0;0;0];
        else
            % Normal operation
            xr  = XrCmd(i,1);  yr  = XrCmd(i,2);  thr = XrCmd(i,3);
            vr  = UrCmd(i,1);  wr  = UrCmd(i,2);
    
            mask = true(1,nR); mask(i) = false;
    
            obsXY = [envXY, posAll(:,mask)];
            obsR  = [envR,  P.robotRadius*ones(1,nnz(mask))];
    
            [dphi_dx, dphi_dy] = potentialGrad(xi, yi, obsXY, obsR, P.robotRadius, P.avoid);
            rep = [dphi_dx; dphi_dy];
    
            [v,w,e,kHat(:,i)] = adaptiveLyapunovController(x, xr, yr, thr, vr, wr, kHat(:,i), P, rep);
    
            if haveLeader && i == leaderIdx
                v = clamp(v, -P.form.vLeadMax, P.form.vLeadMax);
            end
        end

        vAll(i) = v;
        wAll(i) = w;
        Earr(k,:,i) = e.';
    end

    for i = 1:nR
        xi  = Xarr(k,1,i);
        yi  = Xarr(k,2,i);
        thi = Xarr(k,3,i);

        v = vAll(i);
        w = wAll(i);

        Xarr(k+1,1,i) = xi + P.dt * v * cos(thi);
        Xarr(k+1,2,i) = yi + P.dt * v * sin(thi);
        Xarr(k+1,3,i) = wrapToPiCustom(thi + P.dt * w);

        Uarr(k,:,i)    = [v w];
        kHarr(k+1,:,i) = kHat(:,i).';
    end

    allDone = true;
    for i = 1:nR
        if ~isFinished(i)
            allDone = false;
        end
    end
    
    if leaderLocked && isfinite(leaderIdxLock) && isFinished(leaderIdxLock)
        leaderLocked  = false;
        leaderIdxLock = NaN;
    end

    if allDone
        Xarr  = Xarr(1:k+1,:,:);
        Uarr  = Uarr(1:k,:,:);
        Earr  = Earr(1:k,:,:);
        kHarr = kHarr(1:k+1,:,:);

        leaderIdxHist = leaderIdxHist(1:k);
        formOnHist    = formOnHist(1:k);
        degTrigHist   = degTrigHist(1:k,:);

        XrCmdHist     = XrCmdHist(1:k,:,:);
        UrCmdHist     = UrCmdHist(1:k,:,:);

        RcmdHist      = RcmdHist(1:k);
        w0Hist        = w0Hist(1:k);
        vLeadCmdHist  = vLeadCmdHist(1:k);
        dObsMinHist   = dObsMinHist(1:k);
        dClrLeadHist  = dClrLeadHist(1:k);
        
        costHist      = costHist(1:k);
        break;
    end
end

Xsim  = cell(nR,1);
Usim  = cell(nR,1);
Ehist = cell(nR,1);
KhatH = cell(nR,1);

for i = 1:nR
    Xsim{i}  = squeeze(Xarr(:,:,i));
    Usim{i}  = squeeze(Uarr(:,:,i));
    Ehist{i} = squeeze(Earr(:,:,i));
    KhatH{i} = squeeze(kHarr(:,:,i));
end

figure('Color','w'); hold on; axis equal; grid on;
drawEnv(Env);
title('Executed trajectories'); xlabel('x'); ylabel('y');
for i = 1:nR
    plot(Xsim{i}(:,1), Xsim{i}(:,2), '-', 'Color', cols(i,:), 'LineWidth', 2.0);
end

% --- Plot Cost of Formation Graph ---
figure('Color','w');
plot(1:length(costHist), costHist, 'LineWidth', 2, 'Color', 'b');
grid on;
title('Cost of Formation J(t)');
xlabel('Time Step (k)');
ylabel('Cost ($J$)', 'Interpreter', 'latex');
legend('J(t) = N_{active} \cdot C_{base} \cdot e^{\alpha \cdot Progress}');

Ksteps = size(Uarr,1);

P.anim = struct();
P.anim.leaderIdxHist = leaderIdxHist(1:Ksteps);
P.anim.formOnHist    = formOnHist(1:Ksteps);
P.anim.degTrigHist   = degTrigHist(1:Ksteps,:);
P.anim.XrCmdHist     = XrCmdHist(1:Ksteps,:,:);
P.anim.UrCmdHist     = UrCmdHist(1:Ksteps,:,:);
P.anim.ids0          = (0:nR-1).';

P.anim.RcmdHist      = RcmdHist(1:Ksteps);
P.anim.w0Hist        = w0Hist(1:Ksteps);
P.anim.vLeadCmdHist  = vLeadCmdHist(1:Ksteps);
P.anim.dObsMinHist   = dObsMinHist(1:Ksteps);
P.anim.dClrLeadHist  = dClrLeadHist(1:Ksteps);

animateMultiRobot(Env, Xsim, P);
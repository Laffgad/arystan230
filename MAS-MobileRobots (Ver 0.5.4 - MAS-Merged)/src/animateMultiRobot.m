function animateMultiRobot(Env, Xsim, P)

commR = 2.5;  if isfield(P,'commR'), commR = P.commR; end
dTrig = 1.5;  if isfield(P,'dTrig'), dTrig = P.dTrig; end

nR   = numel(Xsim);
ids0 = (0:nR-1).';
if isfield(P,'anim') && isfield(P.anim,'ids0'), ids0 = P.anim.ids0(:); end

cols = lines(nR);

figure('Color','w'); hold on; axis equal; grid on;
drawEnv(Env);
title('Animation (locked leader + formation refs)'); xlabel('x'); ylabel('y');

hPath = gobjects(nR,1);
hBody = gobjects(nR,1);
hHead = gobjects(nR,1);
hTxt  = gobjects(nR,1);
hRef  = gobjects(nR,1);
hRefL = gobjects(nR,1);

for i = 1:nR
    hPath(i) = plot(NaN,NaN,'-','Color',cols(i,:), 'LineWidth', 1.6);
    hBody(i) = rectangle('Position',[0 0 2*P.robotRadius 2*P.robotRadius], ...
        'Curvature',[1 1], 'EdgeColor', cols(i,:), 'LineWidth', 2);
    hHead(i) = quiver(0,0,0,0,0,'LineWidth',2,'MaxHeadSize',2,'Color',cols(i,:));
    hTxt(i)  = text(NaN,NaN,'', 'FontWeight','bold', 'FontSize',11, ...
        'HorizontalAlignment','left', 'VerticalAlignment','bottom', ...
        'Interpreter','none', 'Clipping','on');

    hRef(i)  = plot(NaN,NaN,'o', 'Color', cols(i,:), ...
        'MarkerFaceColor', cols(i,:), 'MarkerSize', 5);
    hRefL(i) = plot(NaN,NaN,':', 'Color', cols(i,:), 'LineWidth', 1.0);
end

th = linspace(0,2*pi,250);
hComm = plot(NaN,NaN,'k:','LineWidth',1.2);
hTrig = plot(NaN,NaN,'k--','LineWidth',1.2);
hForm = plot(NaN,NaN,'k-.','LineWidth',1.3);

pairs = nchoosek(1:nR,2);
nE = size(pairs,1);
hEdge = gobjects(nE,1);
for e = 1:nE
    hEdge(e) = plot(NaN,NaN,'k-','LineWidth',1.0);
end

hInfo = text(0.02,0.98,'', 'Units','normalized', ...
    'FontWeight','bold','FontSize',12, ...
    'HorizontalAlignment','left','VerticalAlignment','top', ...
    'Interpreter','none');

Kmax   = max(cellfun(@(A) size(A,1), Xsim));
stride = 3;

for k = 1:stride:Kmax
    pos = zeros(nR,2);
    thh = zeros(nR,1);

    for i = 1:nR
        Xi = Xsim{i};
        kk = min(k, size(Xi,1));
        pos(i,:) = Xi(kk,1:2);
        thh(i)   = Xi(kk,3);

        set(hPath(i),'XData',Xi(1:kk,1),'YData',Xi(1:kk,2));
        set(hBody(i),'Position',[pos(i,1)-P.robotRadius, pos(i,2)-P.robotRadius, 2*P.robotRadius, 2*P.robotRadius]);
        Lh = 1.2*P.robotRadius;
        set(hHead(i),'XData',pos(i,1),'YData',pos(i,2),'UData',Lh*cos(thh(i)),'VData',Lh*sin(thh(i)));
    end

    % Use stored leader/refs if available
    if isfield(P,'anim') && isfield(P.anim,'leaderIdxHist')
        kkA = min(k, numel(P.anim.leaderIdxHist));
        leaderIdx   = P.anim.leaderIdxHist(kkA);
        formationOn = P.anim.formOnHist(kkA);
        degTrig     = P.anim.degTrigHist(kkA,:).';
        hasRefs     = isfield(P.anim,'XrCmdHist');
    else
        out = selectLeaderByConnections(pos, commR, dTrig, ids0);
        leaderIdx   = out.leaderIdx;
        degTrig     = out.degTrig;
        formationOn = (max(degTrig) > 0);
        hasRefs     = false;
    end

    % adjacency for drawing edges
    D = squareform(pdist(pos));
    commAdj = (D <= commR) & ~eye(nR);
    trigAdj = (D <= dTrig) & commAdj;

    if ~formationOn
        set(hComm,'XData',NaN,'YData',NaN);
        set(hTrig,'XData',NaN,'YData',NaN);
        set(hForm,'XData',NaN,'YData',NaN);

        for e = 1:nE
            set(hEdge(e),'XData',NaN,'YData',NaN);
        end

        for i = 1:nR
            set(hBody(i),'LineWidth',2.0);
            set(hTxt(i),'String',sprintf('R%d[%d]', ids0(i), degTrig(i)));
            set(hTxt(i),'Position',[pos(i,1)+0.15, pos(i,2)+0.15, 0]);
            set(hRef(i),'XData',NaN,'YData',NaN);
            set(hRefL(i),'XData',NaN,'YData',NaN);
        end

        set(hInfo,'String',sprintf('Formation: OFF   (commR=%.2f, dTrig=%.2f)', commR, dTrig));
        drawnow; pause(0.05);
        continue
    end

    Lx = pos(leaderIdx,1); Ly = pos(leaderIdx,2);
    set(hComm,'XData',Lx + commR*cos(th), 'YData',Ly + commR*sin(th));
    set(hTrig,'XData',Lx + dTrig*cos(th), 'YData',Ly + dTrig*sin(th));

    for e = 1:nE
        a = pairs(e,1); b = pairs(e,2);
        if trigAdj(a,b)
            set(hEdge(e),'XData',[pos(a,1) pos(b,1)], 'YData',[pos(a,2) pos(b,2)]);
        else
            set(hEdge(e),'XData',NaN,'YData',NaN);
        end
    end

    dToLeader   = hypot(pos(:,1)-Lx, pos(:,2)-Ly);
    followerMask = (dToLeader <= commR) & ((1:nR).' ~= leaderIdx);

    % formation radius + ref points (from stored XrCmd)
    Rk = NaN;
    if hasRefs
        kkA = min(k, size(P.anim.XrCmdHist,1));
        Rs = [];
        for i = 1:nR
            if followerMask(i)
                rx = P.anim.XrCmdHist(kkA,1,i);
                ry = P.anim.XrCmdHist(kkA,2,i);
                set(hRef(i),'XData',rx,'YData',ry);
                set(hRefL(i),'XData',[pos(i,1) rx],'YData',[pos(i,2) ry]);
                Rs(end+1,1) = hypot(rx-Lx, ry-Ly);
            else
                set(hRef(i),'XData',NaN,'YData',NaN);
                set(hRefL(i),'XData',NaN,'YData',NaN);
            end
        end
        if ~isempty(Rs), Rk = mean(Rs); end
    else
        for i = 1:nR
            set(hRef(i),'XData',NaN,'YData',NaN);
            set(hRefL(i),'XData',NaN,'YData',NaN);
        end
    end

    if isfinite(Rk)
        set(hForm,'XData',Lx + Rk*cos(th), 'YData',Ly + Rk*sin(th));
    else
        set(hForm,'XData',NaN,'YData',NaN);
    end

    for i = 1:nR
        if i == leaderIdx
            set(hBody(i),'LineWidth',3.5);
            set(hTxt(i),'String',sprintf('L%d[%d]', ids0(i), degTrig(i)));
        elseif followerMask(i)
            set(hBody(i),'LineWidth',2.0);
            set(hTxt(i),'String',sprintf('F%d[%d]', ids0(i), degTrig(i)));
        else
            set(hBody(i),'LineWidth',2.0);
            set(hTxt(i),'String',sprintf('R%d[%d]', ids0(i), degTrig(i)));
        end
        set(hTxt(i),'Position',[pos(i,1)+0.15, pos(i,2)+0.15, 0]);
    end

    if isfinite(Rk)
        set(hInfo,'String',sprintf('Formation: ON   Leader: %d   R=%.2f', ids0(leaderIdx), Rk));
    else
        set(hInfo,'String',sprintf('Formation: ON   Leader: %d', ids0(leaderIdx)));
    end

    drawnow;
    pause(0.02);
end
end
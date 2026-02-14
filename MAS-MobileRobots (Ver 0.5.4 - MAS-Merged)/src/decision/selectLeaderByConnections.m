function out = selectLeaderByConnections(pos, commR, dTrig, ids, ignoreMask)

posN = normalizePos(pos);

if nargin < 4 || isempty(ids)
    ids = (1:size(posN,1)).';
else
    ids = ids(:);
end

N = size(posN,1);

% Check for ignoreMask input
if nargin < 5 || isempty(ignoreMask)
    ignoreMask = false(N, 1);
else
    ignoreMask = ignoreMask(:);
    if length(ignoreMask) ~= N
        error('ignoreMask must be size Nx1');
    end
end

D = pairwiseDistances(posN);
commAdj = (D <= commR) & ~eye(N);

% --- Apply ignore mask ---
% If an agent is ignored (finished or previously joined), remove its connections
% This prevents it from being selected as a leader or counted as a neighbor.
commAdj(ignoreMask, :) = false;
commAdj(:, ignoreMask) = false;
% -------------------------

trigAdj = (D <= dTrig) & commAdj;

degTrig = sum(trigAdj, 2);

leaderIdx = NaN;
leaderId  = NaN;

mx = max(degTrig);
if mx > 0
    best = find(degTrig == mx);
    [~, ii] = min(ids(best));
    leaderIdx = best(ii);
    leaderId  = ids(leaderIdx);
end

neighbors = cell(N,1);
for i = 1:N
    neighbors{i} = ids(trigAdj(i,:)).';
end

out = struct();
out.ids       = ids;
out.commAdj   = commAdj;
out.trigAdj   = trigAdj;
out.degTrig   = degTrig;
out.leaderId  = leaderId;
out.leaderIdx = leaderIdx;
out.neighbors = neighbors;

end

function posN = normalizePos(pos)
if size(pos,2) == 2
    posN = pos;
elseif size(pos,1) == 2
    posN = pos.';
else
    error('pos must be Nx2 or 2xN.');
end
end

function D = pairwiseDistances(posN)
N = size(posN,1);
D = zeros(N,N);
for i = 1:N
    for j = i+1:N
        d = hypot(posN(i,1)-posN(j,1), posN(i,2)-posN(j,2));
        D(i,j) = d;
        D(j,i) = d;
    end
end
end
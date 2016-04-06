function [h,tt,yy,ypi] = get_popspike_height(t,y,bt,varargin)
% function h = get_popspike_height(tmsec,y,bounds)
% inputs: tmsec - time in milli sec; y - field potential response trace; bounds = [b1 b2 b3 b4]
% where bi define the part of the trace where the popspike is found.
% [b1 b2] is the time for the boundary of the first bump and [b3
% b4] is the boundary for the second bump. The popspike peak is found
% between these two bumps.
% If manually the three peak points were extraced, then bt is a 2-by-3
% matrix where the first row had the three time points and the second row
% has the three y values.
%
% Outputs: h = height of the popspike
%           tt = time points of the three peaks
%           yy = signal value of the three peaks
%           ypi = 'signal' value at the point of intersection
%
% Example h = get_popspike_height(t,y,[2000 2500 3000 3500])
%
% Mani Subramaniyan
% 2015-09-13
% Here we just manually determine the three peak locations
if size(bt,2)==3
    tt = bt(1,:);
    yy = bt(2,:);
else
    
    args.auto = false; % when true, uses only the first and last bounds to
    % autodetect the popspike bump
    args = parseVarArgs(args,varargin{:});
    
    % Get peaks of the three bumps
    % First smooth the signal
    T = diff(t(1:2));
    sstd = 0.5; % ms std
    sw = getGausswin(sstd,T);
    ys = mconv(y,sw);
    yy = nan(1,3);
    tt = yy;
    large_pos_offset = 1e10; % Add a large positive offset so that when we take absolute
    % value of a signal part of which is negative doesn't get distorted
    np = 10;
    if args.auto
        [~,tind1] = min(abs(t-bt(1)));
        [~,tind2] = min(abs(t-bt(end)));
        ind =tind1:tind2;
        ss = ys(ind);
        st = t(ind);
        so = y(ind);
        slope = diff(ss);
        j = find_zero_crossings(slope,st(2:end));
        nj = length(j);
        if nj == 2
            j = [1 j];
        end
        if nj > 3
            j = j([1 round(nj/2) end]);
            fprintf('Number of zero crossing was %d out of which 3 were chosen\n',nj)           
        end
         nj = length(j);
        for i = 1:nj
            k = [-np np]+j(i);
            kind = k(1):k(2);
            kind = kind(kind>0);
            kind = kind(kind <= length(so));
            sso = so(kind);
            sst = st(kind);
            if slope(j(i)+1) < 0
                % Curved up
                [~,m] = max(abs(sso+large_pos_offset));
            else
                [~,m] = min(abs(sso+large_pos_offset));
            end
            tt(i)= sst(m);
            yy(i) = sso(m);
        end
    else
        
        for i = 1:3
            [~,tind1] = min(abs(t-bt(i)));
            [~,tind2] = min(abs(t-bt(i+1)));
            ind =tind1:tind2;
            ss = ys(ind);
            so = y(ind);
            st = t(ind);
            slope = diff(ss);
            j = find_zero_crossings(slope,st(2:end));
            if ~isempty(j)
                % Check if curvature is upward or downward
                if slope(j(1)+1) < 0
                    % Curved up
                    [~,j] = max(abs(so+large_pos_offset));
                else
                    [~,j] = min(abs(so+large_pos_offset));
                end
                tt(i)= st(j);
                yy(i) = so(j);
            end
        end
    end
end
% Get slope and offset for a line that goes through the first and third
% peak
m = (yy(3)-yy(1))/(tt(3)-tt(1));
c = yy(1)-m*tt(1);

% Y coordinate of point of intersection of the the line the starts from the
% popspike peak and the line that connects that first and last peak
ypi = m*tt(2) + c;

% Popspike amplitude is the distance between the Y-coorinate of the
% popspike peak and the above Y-coordinate of the intersecting point.
h = abs(ypi-yy(2));

% plot(t,y,'k.')
% hold all
%
% plot(tt(2),ypi,'m*')
% plot(tt(2),yy(2),'m*')
% plot(tt([1 3]),yy([1 3]),'k*-')
% plot([tt(2) tt(2)],[yy(2) ypi],'b')
% xlim([4 100])
%
% shg




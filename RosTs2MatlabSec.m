function [sec] = RosTs2MatlabSec(rosTs)
% Convert the ros timestamp to matlab duration type.
ns = uint64(uint64(rosTs.Sec)*1e9 + uint64(rosTs.Nsec));
wholeSecs = floor(double(ns)/1e9);
fracSecs = double(ns - uint64(wholeSecs)*1e9)/1e9;
sec = wholeSecs + fracSecs;
% standardtime = datetime(seconds(sec),'ConvertFrom','posixTime','Format','yyyy.MM.dd HH:mm:ss.SSSSSSSSS');
end


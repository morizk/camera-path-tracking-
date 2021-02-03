function [d] = tdtest(x)
video= VideoReader(x);
frames=zeros(video.Height,video.Width,3);
framecount = 0;
while hasFrame(video)
        framecount = framecount + 1;
        frames=frames+double(readFrame(video));
end
frame1=sum(sum(frames(:,1:floor(video.Width/2),3)));
frame2=sum(sum(frames(:,floor(video.Width/2)+1:end,3)));
re=min(frame1,frame2)/max(frame1,frame2);
if re>.975
    d=3;
else
    d=2;
end


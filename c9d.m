function fout=c9d(vid,d)

%if the user did not enter the type of the video (3d or 2d) this function
%will check if it 2d or 3d 
if nargin == 1
    d=tdtest(vid);
end

% declare variables
benchmark=.48; 
tracker=vision.PointTracker('NumPyramidLevels',5,'MaxBidirectionalError',10);
video= VideoReader(vid);
numframe=video.NumFrames;
framerate=video.FrameRate;
framedu=1/framerate;
numframepermove=round(.1/framedu)*2;
h=video.Height;
if d==2
    w=video.Width;
elseif d==3
    w=(video.Width)/2;
end
t=[0 0 0];
R=[0 0 0];
fout=[0 0 0 0 0 0];

%  default intrinsic matrix of the camera change it if you know your camera intrinsic matrix
focalLength    = [530, 530];    % in units of pixels
principalPoint = [h/2, w/2];    % in units of pixels
imageSize      = [h,w];  % in units of pixels
intrinsics     = cameraIntrinsics(focalLength, principalPoint, imageSize);
frameind=1:floor(numframepermove):numframe;
[~,l]=size(frameind);
ind=1;
% if the video is 2d
if d==2
    %read frame and change it from rgb to gray and apply contrast filter 
    old=rgb2gray(localcontrast((read(video,frameind(ind)))));
    ind=ind+1;
    while(ind<=l)
        new=rgb2gray(localcontrast(read(video,frameind(ind))));
        try
        % get the location of the intersting points in getinterpoint fun
        % differents features detection algorithms - try differets one for
        % your use but thoses worked well for me-
        oldpoints=getinterpoint(old);
        initialize(tracker,oldpoints,old);
        [newpoints,vali]=step(tracker,new);
        %get the features locations in current frame and perverse frame 
        matcho=oldpoints(vali,:);
        matchn=newpoints(vali,:);
        % you can consider this step as filter for the points 
        [fmatrix,epipolar]=estimateFundamentalMatrix(matcho,matchn,'Method','LMedS','NumTrials',1000);
        oipoints=matcho(epipolar,:);
        nipoints=matchn(epipolar,:);
        % calculate the pose of the camera in the crruent frame
        [r,t]=relativeCameraPose(fmatrix,intrinsics,oipoints,nipoints);
        [~,~,rd]=size(r);        
        [td,~]=size(t);
        if td>1
           t=t(end,:); 
        end
         if rd>1
         r=r(:,:,end);
         end
        R=GetEulerAngles(r);
        catch
            %if there was not enough points found in the frames 
            
        end

      showMatchedFeatures(old, new, oipoints, nipoints);

        text(100,100,string([R,t*1.7]))
        drawnow


        drawnow
        old=new;
        ind=ind+1;
        
        release(tracker) 
        prec=ind/l
     fout(end+1,:)=[R,t*1.7];
    end
elseif d==3
   wid=video.Width;
    old=localcontrast(read(video,frameind(ind)));
    old1=rgb2gray(old(:,1:floor(wid/2),:));
    old2=rgb2gray(old(:,floor(wid/2)+1:end,:));
    ind=ind+1;
    while(ind<=l)
        new=localcontrast(read(video,frameind(ind)));
        new1=rgb2gray(new(:,1:floor(wid/2),:));
        new2=rgb2gray(new(:,floor(wid/2)+1:end,:));
        try

        oldpoints1=getinterpoint(old1);
        oldpoints2=getinterpoint(old2);
        initialize(tracker,oldpoints1,old1);
        [newpoints1,vali1]=step(tracker,new1);
        matcho1=oldpoints1(vali1,:);
        matchn1=newpoints1(vali1,:);
        release(tracker) 
        initialize(tracker,oldpoints2,old2);
        [newpoints2,vali2]=step(tracker,new2);
        matcho2=oldpoints2(vali2,:);
        matchn2=newpoints2(vali2,:);
        release(tracker) 
        [fmatrix1,epipolar1]=estimateFundamentalMatrix(matcho1,matchn1,'Method','LMedS','NumTrials',800);
        oipoints1=matcho1(epipolar1,:);
        nipoints1=matchn1(epipolar1,:);
        [fmatrix2,epipolar2]=estimateFundamentalMatrix(matcho2,matchn2,'Method','LMedS','NumTrials',800);
        oipoints2=matcho2(epipolar2,:);
        nipoints2=matchn2(epipolar2,:);
        [r1,t1,q1]=relativeCameraPose(fmatrix1,intrinsics,oipoints1,nipoints1);
        [r2,t2,q2]=relativeCameraPose(fmatrix2,intrinsics,oipoints2,nipoints2);

        [~,~,rd1]=size(r1);        
        [td1,~]=size(t1);
        
         [~,~,rd2]=size(r2);        
        [td2,~]=size(t2);
        if td1>1
           t1=t1(end,:); 
        end
         if rd1>1
         r1=r1(:,:,end);
         end
         
         if td2>1
           t2=t2(end,:); 
         end
         if rd2>1
         r2=r2(:,:,end);
         end
         
        R1=GetEulerAngles(r1);
        R2=GetEulerAngles(r2);
        % check if the quality of the points is better the the benchmark -
        % max is 1-  and average the predictions of each half depending on
        % the quality of the points 
            if q1 > benchmark || q2 >benchmark
            if q1<benchmark
                q1=0;
            end
            if q2<benchmark
                q2=0;
            end 
            R=(q1*R1+q2*R2)/(q1+q2);
            t=(q1*t1+q2*t2)/(q1+q2);
            else

            end
        catch

        end
        

subplot(1,2,1);
showMatchedFeatures(old1, new1, oipoints1, nipoints1);
subplot(1,2,2);
showMatchedFeatures(old2, new2, oipoints2, nipoints2);



        text(100,100,string([R,t*1.7]))
        drawnow
        old1=new1;
        old2=new2;
        ind=ind+1;
        prec=ind/l
        fout(end+1,:)=[R,t*1.7];
   end

end


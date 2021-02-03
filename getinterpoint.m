function points=getinterpoint(image)
        oldpoints1=detectSURFFeatures(image,'MetricThreshold',900,'NumScaleLevels' ,5);
%         oldpoints11=detectBRISKFeatures(image);
        oldpoints2=detectHarrisFeatures(image);
        points=unique(round([oldpoints1.Location;oldpoints2.Location]),'row');

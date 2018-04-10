%concatenator
%concatenates take_off.txt, cruising.txt, landing.txt
load take_off.txt;
load cruising.txt;
load landing.txt;

%target filename specified is path.txt
%all values should have three decimal points
dlmwrite('path.txt',take_off,'delimiter',' ','precision','%.3f')
dlmwrite('path.txt',cruising,'-append','delimiter',' ','precision','%.3f') 
dlmwrite('path.txt',landing,'-append','delimiter',' ','precision','%.3f') 
%path.txt looks messy, but matlab and sublime detect 11 columns
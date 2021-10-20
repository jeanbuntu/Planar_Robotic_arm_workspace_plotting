%The function pushes out out an array with coordinates of the workspace of
%a 3DOF planar 3 link robot. This is done with an input resolution against 
%the input link length and joint angle arrays.
%Link Length array: A_arr = [Length1 Length2 Length3]
%Joint angle array: theta_arr = [j1min j1max j2min j2max j3min j3max]
%Coded in/for MATLAB R2021a

%Please do feel free to contact me with
%doubts or errors at "jeano5326@gmail.com" I will be more than glad to
%hear any input on these. 

% https://www.linkedin.com/in/jeanojoseph/
% https://www.behance.net/jeanojoseph
% https://github.com/jeanbuntu
% @ - jeano5326@gmail.com

function [xytheta_arr,k] = Wspace_mod(A_arr,theta_arr,res)
%%
aone = A_arr(1); 
a2 = A_arr(2);
a3 = A_arr(3); 

%%
numPt = res; 
numPt_3 = numPt * numPt * numPt;

%%
q1 = linspace(theta_arr(1),theta_arr(2),numPt); %(-pi/3,pi/3,numPt);
q2 = linspace(theta_arr(3),theta_arr(4),numPt); %(-2*pi/3,2*pi/3,numPt);
q3 = linspace(theta_arr(5),theta_arr(6),numPt); %(-pi/2,pi/2,numPt);

%%
%X_base =0; Y_base = 0;
%pause(2)
%max_arr = ones(2,numPt);

%%
%declaring array size for final storage before push
x_stor = ones(1,numPt_3); 
y_stor = ones(1,numPt_3);
angle_stor = ones(numPt_3,3);

storage_i = 1;
            
%%            
%3 nested loops for 3 respective joints.
for i=1:numPt 
    for j=1:numPt        
        for k = 1 : numPt 
            %X,Y coordinates calculated by the projections of the
            %instantaneous joint positions for respective angles.
            xcord = aone*cos(q1(i)) + a2*cos(q1(i)+q2(j))+a3*cos(q1(i)+q2(j)+q3(k));
            ycord = aone*sin(q1(i)) + a2*sin(q1(i)+q2(j))+a3*sin(q1(i)+q2(j)+q3(k)); 
            
            x_stor(storage_i) = xcord; %stores the X coordinates in column 1
            y_stor(storage_i) = ycord; % Y coordinates in column 2
            angle_stor(storage_i,:) = [q1(i) q2(j) q3(k)]; %joint angles j1 j2 and j3 respectively in
            %columns 3,4 and 5.
            storage_i = storage_i + 1;
       end 
    end
end
xytheta_arr = [x_stor' y_stor' angle_stor];
%%
k = boundary(xytheta_arr(:,1),xytheta_arr(:,2),0.75);
hold on;
%plot(xytheta_arr(k,1),xytheta_arr(k,2),'.'); 
%uncomment the above line and call the function to highlight the edge
%coordinates
plot(xytheta_arr(:,1),xytheta_arr(:,2),'.','MarkerEdgeColor','k');
end
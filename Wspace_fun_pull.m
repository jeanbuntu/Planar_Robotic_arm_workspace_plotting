%The code pulls the data points from the Wspace_mod function, with inputs
%here on Link lengths and respective joint actuation domains. The output
%array from Wspace_mod is an array CA contatining workspace span coordinates 
%at the input resolution and the column vector maxmin containing the boundary
%coordinates of the span which is then simulated with the joints in the
%code below.

%Personally, I would recommend utilising resolutions in the range of 10<res<30. 
%Any more will absolutely work at the cost of more
%time, but this code hasn't been designed for that degree of analysis and
%just for mere eye leve observation. Please do feel free to contact me with
%doubts or errors at "jeano5326@gmail.com" I will be more than glad to
%hear any input on these. 
%Coded in/for MATLAB R2021a

% https://www.linkedin.com/in/jeanojoseph/
% https://www.behance.net/jeanojoseph
% https://github.com/jeanbuntu
% @ - jeano5326@gmail.com

%%
%constant inputs
rse = 30; %resolution for data point density
Ang_arr = [0 pi/3 0 2*pi/3 0 pi/2]; %Angle array containing joint angle limits
%[j1_tmin j1_tmax j2_tmin j2_tmax j3_tmin j3_tmax]
%angular domain in radians 

L_arr = [0.2 0.2 0.2]; %Link length array with respective joint lengths
%[len1 len2 len3]

[CA,maxmin] = Wspace_mod(L_arr, Ang_arr, rse);

%%
numPt = size(maxmin);
X_base = 0;
Y_base = 0;

for i=1:numPt 
    
    
    base = plot(X_base,Y_base,'o','MarkerSize',5,'MarkerEdgeColor','k','MarkerFaceColor','k');
    %%setting figure initial properties
    axis padded
    pbaspect([1 1 1])
    hold on
    
    %%declaring angle variables
    q1 = CA(maxmin(i),3);
    q2 = CA(maxmin(i),4);
    q3 = CA(maxmin(i),5);
    
    %declaring X,Y coordinates for Joint 1 wrt base
    A1_x_tip = 0 + (L_arr(1)*cos(q1));
    A1_y_tip = 0 + (L_arr(1)*sin(q1));
    L1 = plot([X_base A1_x_tip],[Y_base A1_y_tip],'r','LineWidth',2);
    
    %declaring X,Y coordinates for Joint 2 wrt base(i.e. joint 1)
    A2_x_tip=A1_x_tip+(L_arr(2)*cos(q1+q2));
    A2_y_tip=A1_y_tip+(L_arr(2)*sin(q1+q2));
    L2 = plot([A1_x_tip A2_x_tip],[A1_y_tip A2_y_tip],'g','LineWidth',2);

    %declaring X,Y coordinates for Joint 3 wrt base(i.e. joint 2)
    A3_x_tip=A2_x_tip+(L_arr(3)*cos(q1+q2+q3));
    A3_y_tip=A2_y_tip+(L_arr(3)*sin(q1+q2+q3));
    L3 = plot([A2_x_tip A3_x_tip],[A2_y_tip A3_y_tip],'b','LineWidth',2);
    
    %Marking the End Effector separately
    E_eff = plot(A3_x_tip, A3_y_tip,'s','MarkerSize',5,'MarkerEdgeColor','c','MarkerFaceColor','c');
    
    %Plot properties
    pause(0.1)
    title('Workspace Perimeter Sweep')
    xlabel('X Coordinates')
    ylabel('Y Coordinates')
    
    %a simple clause to prevent the arms from being deleted from the
    %figure in the final iteration
    if i < numPt(1)
        delete(L1); delete(L2); delete(L3); delete(base); %delete(E_eff);
    end
end 
function [T] = trans_matrix(A, start, last)
T=eye(4);
while(last ~=  start)
  
    ct=cosd(A(start+1,4));
    st=sind(A(start+1,4));
    sa=sind(A(start+1,2));
    ca=cosd(A(start+1,2));
    a=(A(start+1,1));
    d=A(start+1,3);
    T = T*[ct,-st*ca,st*sa,a*ct;
        st,ct*ca,-ct*sa,a*st;
        0,sa,ca,d;
        0,0,0,1];
    start=start+1;
    
    
end
end


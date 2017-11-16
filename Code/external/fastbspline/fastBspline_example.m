 %Fit a noisy measurement with a smoothness-penalized spline (p-spline)
    x = (0:.5:10)';
    y = sin(x*pi*.41-.9)+randn(size(x))*.2;
    knots = [0,0,0,0:.5:10,10,10,10]; 
    %Notice there are as many knots as observations
    
    %Because there are so many knots, this is an exact interpolant
    sp1 = fastBSpline.lsqspline(knots,3,x,y);
    %Fit penalized on the smoothness of the spline
    sp2 = fastBSpline.pspline(knots,3,x,y,.7);
    
    clf;
    rg = -2:.005:12;
    plot(x,y,'o',rg,sp1.evalAt(rg),rg,sp2.evalAt(rg));
    legend('measured','interpolant','smoothed');
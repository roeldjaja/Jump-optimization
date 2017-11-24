% Calculate references for joints
% Leg: q = [x1, y1, theta, q1, q2, q3]
q_ref       = zeros(6,length(t));
q_ref(1,:)  = 0;
q_ref(2,:)  = 0;
q_ref(3,:)  = 0;

% Timing stages, time span in seconds
stg1 = 0.1; stg2 = 0.3; stg3 = 0.4;

% Stage 1
if (0<=t) & (t<=stg1)
    q_ref(4,:)  = -0.5;
    q_ref(5,:)  = 1.6;
    q_ref(6,:)  = -1.2;
end

% Stage 2: Hip, knee and ankle extension
if (stg1<t) & (t<=(stg1+stg2))
    q_ref(4,:)  = -0.7 ;
    q_ref(5,:)  = 1.5 ;
    q_ref(6,:)  = -1.2 ;
end


% Stage 3:  push off
if (stg1+stg2<t) & (t<=(stg1+stg2+stg3))
    q_ref(4,:)  = 0.8;
    q_ref(5,:)  = 0.6;
    q_ref(6,:)  = -0.2;
end

% Stage 4: Fly
if (stg1+stg2+stg3<t)
    q_ref(4,:)  = -0.4;
    q_ref(5,:)  = 0.2;
    q_ref(6,:)  = 0;
end
this.ref.henk =q_ref;
% Calculate reference velocities
q_d_ref = diff(q_ref')' / this.control.Ts;
q_d_ref = [q_d_ref(:,1), q_d_ref];



%__________________________________________________________________

data =opt.data.qinit';

lessdata4 = data(4,1:100:end);
lessdata5 = data(5,1:100:end);
lessdata6 = data(6,1:100:end);

% yy = spline(x,y,xx);
x = 0:0.1:1;
xx = linspace(0,1,1001);
q_init = zeros(6,1001);

y4 = lessdata4;
cs4 = spline(x,[data(4,1) y4 data(4,end)]);
y5 = lessdata5;
cs5 = spline(x,[data(5,1) y5 data(5,end)]);
y6 = lessdata6;
cs6 = spline(x,[data(6,1) y6 data(6,end)]);

q_init(4,:) = ppval(xx,cs4);
q_init(5,:) = ppval(xx,cs5);
q_init(6,:) = ppval(xx,cs6);


save('q_init','q_init')
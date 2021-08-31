function [t0,x0 ,u0] = shift_mv_obs(T, t0,x0,u, mv_obs_velo_ms, mv_obs_theta_rad, f)
st = x0;
con = u(1,:)'; 
f_value = f(st,con, mv_obs_velo_ms, mv_obs_theta_rad);   
st = st+ (T*f_value);
x0 = full(st);

t0 = t0 + T;
u0 = [u(2:size(u,1),:);u(size(u,1),:)]; % shift the control action 
end
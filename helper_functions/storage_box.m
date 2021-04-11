function box = storage_box(N,varargin)

if nargin > 1
    n = varargin{1};
else
    n = 1;
end

%
for i = 1:n
    box(i).t  = zeros(1,N);

    %% measurements
    box(i).z.ideal  = zeros(3,N);
    box(i).z.wNoise = zeros(6,N);

    %% estimates
    box(i).x.true = zeros(6,N);
    box(i).x.est  = zeros(6,N); 

    %% variances
    box(i).P.est = zeros(6,6,N); 

    %% constraints 
    box(i).const.prior_update  = zeros(2,N); 
    box(i).const.meas_update   = zeros(2,N); 
    box(i).const.corr_update   = zeros(2,N); 

end

end
function[] = rectify()
[filename1,filepath1]=uigetfile({'Image Files'},...
'Select Data File 1');
cd(filepath1);
fp= fopen(filename1);
%%% WORLD
x1(:,1) = [0 0 1]';
x1(:,2) = [200 0 1]';
x1(:,3) = [200 100 1]';
x1(:,4) = [0 100 1]';
%%% IMAGE
im = imread(filename1);
imshow(im);
hold on;
x2 = zeros( size(x1) );
for k = 1 : 4
p = ginput(1);
plot( p(1), p(2), 'yo' );
x2(1:2,k) = p;
x2(3,k) = 1;
end
hold off;
%%% HOMOGRAPHY
H = homography( x1, x2 );
H = H / H(9);
%%% RECTIFY
T1 = maketform( 'projective', inv(H)');
im2 = imtransform(im, T1);
imshow( im2 );
set( gca, 'Ydir', 'normal' );
%%% ------------------------------------------------------------
function[H] = homography(x1,x2)
[x1, T1] = normalise(x1); % WORLD COORDINATES
[x2, T2] = normalise(x2); % IMAGE COORDINATES
N = length(x1);
A = zeros(3*N,9);
O = [0 0 0];
for n = 1 : N
X = x1(:,n)';
x = x2(1,n);
y = x2(2,n);
s = x2(3,n);
A(3*n-2,:) = [ O -s*X y*X];
A(3*n-1,:) = [ s*X O -x*X];
A(3*n ,:) = [-y*X x*X O ];
end
[U,D,V] = svd(A,0); % TOTAL LEAST-SQUARES
H = reshape(V(:,9),3,3)';
H = inv(T2)*H*T1;
%%% ------------------------------------------------------------
function[P2,T] = normalise(P)
P(1,:) = P(1,:)./P(3,:);
P(2,:) = P(2,:)./P(3,:);
P(3,:) = 1;
c = mean(P(1:2,:)')'; % TRANSLATE
P1(1,:) = P(1,:) - c(1);
P1(2,:) = P(2,:) - c(2);
dist = sqrt(P1(1,:).^2 + P1(2,:).^2); % SCALE
scale = sqrt(2)/mean(dist);
T = [scale 0 -scale*c(1)
0 scale -scale*c(2)
0 0 1 ];
P2 = T*P; % NORMALIZE POINTS
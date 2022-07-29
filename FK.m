% q1 q2
q1 = 0.1;
q2 = 0.3;

l1 = 0.3;
l2 = 0.3;
pA = [0;0];
pB = pA + Rot(q1) * [0; -l1];
pC = pB + Rot(q2) * [0; -l2];

plot([pA(1),pB(1),pC(1)], [pA(2),pB(2),pC(2)], 'LineWidth',2, 'color','k');
axis equal

%%
function R = Rot(q)
    R = [cos(q), -sin(q);
         sin(q),  cos(q)];
end
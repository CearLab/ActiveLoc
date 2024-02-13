clear 
syms a b c d f g

R = [   a^2+b^2+c^2,    -a^2,   -b^2,   -c^2;   ...
        -a^2,   a^2+d^2+f^2,    -d^2,   -f^2;   ...
        -b^2,   -d^2,   b^2+d^2+g^2,    -g^2;   ...
        -c^2,   -f^2,   -g^2,     c^2+f^2+g^2   ];

e = simplify(eig(R));

syms A B C D
e = simplify(subs(e,a^2+b^2+c^2,A));
e = simplify(subs(e,a^2+d^2+f^2,B));
e = simplify(subs(e,b^2+d^2+g^2,C));
e = simplify(subs(e,c^2+f^2+g^2,D));

int orderint = (int) order;
int ii;int jj;        
for(jj = 0; jj < x_length; jj++)
{
    double s = 0;
    for(ii = firstknot[jj]; ii < lastknot[jj]; ii++)
    {
        s += weights[ii-1]*bin(&(knots[-1]),ii,orderint,x[jj]);
    }
    Sx[jj] = s;
}


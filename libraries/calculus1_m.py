import time

def track(n):
    graph_x=[]
    for num in range(0,n):
        graph_x.append(str('0'))
    return graph_x

def update(graph_x,x):
    if(graph_x[-1]=='0'):
        i=0
        for value in graph_x:
            if(value == '0'):
                graph_x[i]=x
                break
            i+=1
    else:
        graph_x.pop(0)
        graph_x.append(x)
    return(graph_x)

def der(graph_x,graph_y,n):
    if(graph_x[n]!='0' and graph_y[n]!='0'):
        d=(graph_x[n]-graph_x[n-1])/(graph_y[n]-graph_y[n-1])
        return d
    else:
        return 'error'

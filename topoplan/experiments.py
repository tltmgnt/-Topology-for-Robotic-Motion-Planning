"""Strict, bounded experiment contract and backend dispatch; legacy Grid unchanged."""
import copy
import platform
import time
from .core import Grid
from .geometry import number, point, clearance_sensitivity
from .continuous import rrt
from .temporal import timed_plan, prioritized_plan, validate_task, verify_timed_path
from .audit import analyze_static, reference_timed

PLANNERS=('astar','rrt','ompl:PRM','ompl:RRT','ompl:RRTstar')


def fields(value,required,optional=()):
    if not isinstance(value,dict) or not set(required)<=set(value) or set(value)-set(required)-set(optional):
        raise ValueError('missing or unknown experiment fields')


def integer(value,low,high):
    if type(value) is not int or not low<=value<=high:raise ValueError('bounded integer required')
    return value


def normalize(study):
    fields(study,{'schema_version','experiments'})
    if type(study['schema_version']) is not int or study['schema_version']!=1:raise ValueError('study schema version 1 required')
    experiments=study['experiments']
    if not isinstance(experiments,list) or not 1<=len(experiments)<=64:raise ValueError('one to 64 experiments required')
    result=[];ids=set();total_states=0;rrt_work=0
    for raw in experiments:
        if not isinstance(raw,dict):raise ValueError('experiment object required')
        kind=raw.get('kind')
        common={'id','kind','grid'};optional={'radius','max_expansions'}
        if kind=='static':
            fields(raw,common|{'start','goal'},optional|{'planner','routes','iterations','seed','seconds','coordinate_mode'})
        elif kind=='dynamic':fields(raw,common|{'start','goal'},optional|{'obstacles','horizon'})
        elif kind=='multi':fields(raw,common|{'robots'},optional|{'obstacles','horizon'})
        else:raise ValueError('unknown experiment kind')
        name=raw['id']
        if not isinstance(name,str) or not name.strip() or name!=name.strip() or len(name)>100 or name in ids:raise ValueError('unique trimmed experiment ID required')
        ids.add(name);exp=copy.deepcopy(raw);exp['radius']=number(exp.get('radius',0))
        grid=Grid(exp['grid'],exp['radius'])
        if grid.width*grid.height>2500 or len(grid.blocked)>500:raise ValueError('benchmark bounded to 2500 cells and 500 occupied cells')
        exp['max_expansions']=integer(exp.get('max_expansions',100000),1,200000)
        if kind=='static':
            exp['planner']=exp.get('planner','astar')
            if exp['planner'] not in PLANNERS:raise ValueError('unknown planner')
            exp['routes']=integer(exp.get('routes',2),1,10)
            exp['iterations']=integer(exp.get('iterations',1000),1,10000)
            if exp['planner']=='rrt':
                rrt_work+=exp['iterations']**2
                if rrt_work>25000000:raise ValueError('continuous RRT nearest-neighbor study budget exceeded')
            exp['seed']=integer(exp.get('seed',0),0,2**32-1)
            exp['seconds']=number(exp.get('seconds',.2),.001,10)
            exp['coordinate_mode']=exp.get('coordinate_mode','cells')
            if exp['coordinate_mode'] not in ('cells','world'):raise ValueError('coordinate mode must be cells or world')
            if exp['coordinate_mode']=='cells':grid.point(exp['start']);grid.point(exp['goal'])
            else:
                if exp['planner']=='astar':raise ValueError('A* requires cell coordinates')
                point(exp['start']);point(exp['goal'])
            if exp['routes']>1 and len(grid.markers)>6:raise ValueError('at most six obstacle components for diversity studies')
            states=grid.width*grid.height*(3**len(grid.markers) if exp['planner']=='astar' and exp['routes']>1 else 1)
        else:
            exp['horizon']=integer(exp.get('horizon',32),0,128);exp['obstacles']=exp.get('obstacles',[])
            states=grid.width*grid.height*(exp['horizon']+1)*(len(exp['robots']) if kind=='multi' and isinstance(exp['robots'],list) else 1)
        if states>200000:raise ValueError('experiment proof state budget exceeded')
        total_states+=states
        if total_states>1000000:raise ValueError('study proof budget exceeds one million states')
        result.append(exp)
    return {'schema_version':1,'experiments':result}


def execute(exp):
    grid=Grid(exp['grid'],exp['radius']);start=time.perf_counter()
    try:
        if exp['kind']=='static':
            a,b=exp['start'],exp['goal']
            if exp['planner']=='astar':
                report=grid.plan(a,b,exp['routes'],exp['max_expansions'])
                result={**report,'termination':report['status']}
            else:
                if exp['coordinate_mode']=='cells':a=[v+.5 for v in a];b=[v+.5 for v in b]
                if exp['planner']=='rrt':result=rrt(grid,a,b,exp['routes'],exp['iterations'],exp['seed'])
                else:
                    from .baselines import ompl_plan
                    result=ompl_plan(grid,a,b,exp['planner'].split(':')[1],exp['routes'],exp['seconds'],exp['seed'])
        elif exp['kind']=='dynamic':
            result=timed_plan(grid,exp['start'],exp['goal'],exp['obstacles'],exp['horizon'],exp['max_expansions'])
        else:
            result=prioritized_plan(grid,exp['robots'],exp['horizon'],exp['max_expansions'],exp['obstacles'])
    except ValueError as error:
        result={'termination':'invalid_input','error':str(error)}
    return {'id':exp['id'],'result':result,'wall_seconds':time.perf_counter()-start,
            'runtime':{'python':platform.python_version(),'implementation':platform.python_implementation()}}


def check_experiment(exp,receipt):
    fields(receipt,{'id','result','wall_seconds','runtime'})
    fields(receipt['runtime'],{'python','implementation'})
    if any(not isinstance(v,str) or not 1<=len(v)<=100 for v in receipt['runtime'].values()):
        raise ValueError('bounded runtime version strings required')
    if receipt['id']!=exp['id']:raise ValueError('receipt ID mismatch')
    number(receipt['wall_seconds'],0,1e6)
    result=receipt['result'];grid=Grid(exp['grid'],exp['radius'])
    if not isinstance(result,dict):raise ValueError('result object required')
    term=result.get('termination')
    # Deterministic internal planners are replayed; timing observations and native OMPL outputs are not regenerated.
    if exp['kind']!='static' or exp['planner'] in ('astar','rrt'):
        replay=execute(exp)['result']
        if replay!=result:raise ValueError('deterministic planner replay mismatch')
    elif term=='invalid_input':
        from .geometry import Space
        endpoints=[point(exp[k]) for k in ('start','goal')]
        if exp['coordinate_mode']=='cells':endpoints=[tuple(v+.5 for v in p) for p in endpoints]
        if all(Space(grid).valid(p) for p in endpoints):raise ValueError('false invalid endpoint claim')
    if term=='invalid_input':return {'termination':term,'verification':{'input_invalidity_verified':True}}
    if exp['kind']=='static':
        allowed={'requested_routes_found','bounded_search_exhausted','expansion_limit'} if exp['planner']=='astar' else {'requested_routes_found','iteration_limit'} if exp['planner']=='rrt' else {'requested_routes_found','time_or_attempt_limit','backend_unavailable','backend_error','backend_timeout'}
        if term not in allowed:raise ValueError('unsupported termination claim')
        if exp['planner'].startswith('ompl:') and result.get('exhaustive') is not False:
            raise ValueError('native sampling planner cannot claim exhaustive search')
        routes=result.get('routes')
        if not isinstance(routes,list) or len(routes)>exp['routes']:raise ValueError('route count mismatch')
        if (term=='requested_routes_found') != (len(routes)>=exp['routes']):raise ValueError('requested route completion mismatch')
        return {'termination':term,**analyze_static(grid,exp,result)}
    if exp['kind']=='dynamic':
        tasks=[({'start':exp['start'],'goal':exp['goal']},result)]
    else:
        tasks=list(zip(exp['robots'],result['robots']))
    reservations=list(exp['obstacles']);proofs=[]
    for robot,report in tasks:
        if report['path'] is not None:
            verify_timed_path(grid,report['path'],robot['start'],robot['goal'],exp['horizon'],reservations)
        earliest,states=reference_timed(grid,robot['start'],robot['goal'],exp['horizon'],reservations)
        if report['termination']=='horizon_exhausted' and earliest is not None:raise ValueError('false horizon exhaustion')
        if report['path'] is not None and report['arrival_tick']!=earliest:raise ValueError('timed route arrival not optimal under reservations')
        proofs.append({'route_checked':report['path'] is not None,'exhaustion_verified':report['termination']=='horizon_exhausted',
                       'earliest_feasible_arrival':earliest,'reference_states':states})
        if report['path'] is not None:reservations.append({'path':report['path'],'radius':grid.radius})
    return {'termination':term,'verification':proofs,'joint_infeasibility_claimed':False}


def summarize(study,receipts):
    if not isinstance(receipts,list) or len(receipts)!=len(study['experiments']):raise ValueError('one receipt per experiment required')
    results=[];diversity={};sweeps={}
    for exp,receipt in zip(study['experiments'],receipts):
        analysis=check_experiment(exp,receipt)
        results.append({'id':exp['id'],'kind':exp['kind'],'planner':exp.get('planner'),
                        'radius':exp['radius'],'wall_seconds':receipt['wall_seconds'],'runtime':receipt['runtime'],**analysis})
        if exp['kind']=='static':
            key=(tuple(exp['grid']),tuple(exp['start']),tuple(exp['goal']),exp['coordinate_mode'],exp['radius'],exp['planner'])
            group=diversity.setdefault(key,{'experiment_ids':[],'signatures':set(),'paths':set()})
            group['experiment_ids'].append(exp['id'])
            for route in receipt['result'].get('routes',[]):
                group['signatures'].add(tuple(route['signature']));group['paths'].add(tuple(tuple(p) for p in route['path']))
            rows=tuple(exp['grid']);sweeps.setdefault(rows,set()).add(exp['radius'])
    return {'schema_version':1,'engine_version':'0.2.0','experiments':results,
            'diversity_groups':[{'experiment_ids':g['experiment_ids'],'distinct_signatures':len(g['signatures']),
                                 'distinct_waypoint_sequences':len(g['paths'])} for g in diversity.values()],
            'clearance_sweeps':[{'grid':list(rows),'observations':clearance_sensitivity(list(rows),sorted(radii))} for rows,radii in sweeps.items()],
            'execution_authorized':False,'source_authenticated':False,
            'limitations':['Signatures distinguish some homology classes, not complete homotopy classes',
                           'Finite grid/horizon exhaustion does not establish continuous or joint multi-robot infeasibility',
                           'Budgets have different units; no equal-compute or performance-superiority claim',
                           'Wall time is a captured observation, not independently reconstructed or authenticated',
                           'Native OMPL routes are checked offline; native searches are not replayed',
                           'Static conservative expanded-square geometry; timed disks follow supplied trajectories',
                           'No perception model, high-DoF manipulation, kinodynamics, hardware execution or safety certification']}

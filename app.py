from flask import Flask, request, jsonify
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from flask_cors import CORS

# --- 1. إعداد تطبيق Flask ---
app = Flask(__name__)
# للسماح للواجهة الأمامية (index.html) بالتواصل مع هذا الخادم
CORS(app) 

# --- 2. دالة توليد مصفوفة المسافات الافتراضية ---
# في تطبيق حقيقي، سيتم استبدال هذه الدالة باستدعاء Google Maps API
def create_distance_matrix(num_locations):
    """توليد مصفوفة مسافات افتراضية (المسافات بالكيلومتر * 1000)."""
    # يجب أن تكون مصفوفة متماثلة
    # 0 = المستودع
    if num_locations <= 1:
        return [[0]]
        
    # توليد مصفوفة مسافات افتراضية مربعة
    matrix = []
    import random
    random.seed(42) # لتكون النتائج قابلة للتكرار

    for i in range(num_locations):
        row = []
        for j in range(num_locations):
            if i == j:
                row.append(0)
            elif j < i:
                # ضمان التماثل (Distance(i, j) = Distance(j, i))
                row.append(matrix[j][i])
            else:
                # مسافات عشوائية بين 500 متر و 50 كم (مضروبة في 1000 لتمثيلها كأعداد صحيحة)
                distance_in_meters = random.randint(500, 50000)
                row.append(distance_in_meters)
        matrix.append(row)
    
    return matrix

# --- 3. دالة التحسين الرئيسية ---
def solve_vrp(data):
    """حل مشكلة توجيه المركبات مع قيود السعة (CVRP)."""
    
    manager = pywrapcp.RoutingIndexManager(
        data['num_locations'], data['num_vehicles'], data['depot_index'])
    routing = pywrapcp.RoutingModel(manager)

    # 3.1. دالة التكلفة (المسافة)
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # 3.2. قيود السعة (الحمولة)
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,                      # سعة الانتظار (slack)
        data['vehicle_capacities'],
        True,                   # البدء من الصفر
        'Capacity')

    # 3.3. معايير البحث
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = 5 # إعطاء وقت أطول للبحث عن حل أفضل

    # 3.4. تشغيل الحل
    solution = routing.SolveWithParameters(search_parameters)

    return routing, manager, solution

# --- 4. دالة عرض النتائج (Processing the Solution) ---
def get_solution_details(data, manager, routing, solution):
    """تحويل كائن الحل إلى تنسيق JSON."""
    if not solution:
        return {'solution_status': 'NO_SOLUTION', 'routes': []}

    routes = []
    capacity_dimension = routing.GetDimensionOrDie('Capacity')
    
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        route_nodes = []
        route_distance = 0
        
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            
            # إضافة العميل فقط، وتخطي المستودع
            if node_index != data['depot_index']:
                 route_nodes.append(node_index)

            previous_index = index
            index = solution.Value(routing.NextVar(index))
            
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            
        # الحمولة الكلية المستخدمة (أقصى حمولة تم حملها في المسار)
        end_index = routing.End(vehicle_id)
        total_load = solution.Value(capacity_dimension.CumulVar(end_index))
        
        routes.append({
            'vehicle_id': vehicle_id + 1,
            'route_nodes': route_nodes,
            # قسمة المسافة على 1000 لعرضها بالكيلومتر
            'total_distance': route_distance / 1000, 
            'total_load': total_load
        })

    return {
        'solution_status': 'OPTIMAL',
        'objective_value': solution.ObjectiveValue() / 1000,
        'routes': routes
    }


# --- 5. نقطة نهاية API ---
@app.route('/solve_vrp', methods=['POST'])
def handle_vrp_request():
    try:
        request_data = request.get_json()
        
        customer_data = request_data.get('customer_data', [])
        vehicle_capacities = request_data.get('vehicle_capacities', [])
        depot_index = request_data.get('depot_index', 0)

        # 1. تجهيز البيانات
        num_customers = len(customer_data)
        num_locations = num_customers + 1
        num_vehicles = len(vehicle_capacities)
        
        # التأكد من أن العملاء يبدأون من ID=1 بعد المستودع ID=0
        demands = [0] * num_locations
        for row in customer_data:
            loc_id = int(row['LocationID'])
            demand = int(row['Demand'])
            if 0 < loc_id < num_locations:
                demands[loc_id] = demand
            else:
                raise ValueError(f"LocationID {loc_id} غير صالح. يجب أن يكون بين 1 و {num_customers}.")

        data = {
            'distance_matrix': create_distance_matrix(num_locations), 
            'demands': demands,
            'num_locations': num_locations,
            'num_vehicles': num_vehicles,
            'depot_index': depot_index,
            'vehicle_capacities': vehicle_capacities
        }

        # 2. حل المشكلة
        routing, manager, solution = solve_vrp(data)

        # 3. إعداد الاستجابة
        response = get_solution_details(data, manager, routing, solution)
        return jsonify(response)

    except Exception as e:
        print(f"Error during VRP solving: {e}")
        return jsonify({'error': str(e), 'solution_status': 'ERROR'}), 500

# --- 6. تشغيل الخادم ---
if __name__ == '__main__':
    app.run(debug=True)
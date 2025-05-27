# pip install -U langchain langchain-community langchain-core langchain-text-splitters langchain-ollama chromadb beautifulsoup4 tiktoken pypdf lxml matplotlib
import os
import glob
from typing import List, Union, Optional

from langchain_community.document_loaders import WebBaseLoader, BSHTMLLoader, PyPDFLoader
from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_ollama import OllamaEmbeddings 
from langchain_community.vectorstores import Chroma
from langchain_core.output_parsers import StrOutputParser
from langchain.prompts import ChatPromptTemplate
from langchain_ollama import ChatOllama
from langchain_core.runnables import RunnablePassthrough
from langchain_core.documents import Document
import sys # Add this import


import re

current_script_dir = os.path.dirname(os.path.abspath(__file__))
# Navigate up to the 'my_scripts' directory
# assignment_3_dir is os.path.dirname(current_script_dir) -> .../assignment_3/
# my_scripts_dir is os.path.dirname(os.path.dirname(current_script_dir)) -> .../my_scripts/
my_scripts_dir = os.path.dirname(os.path.dirname(current_script_dir))
if my_scripts_dir not in sys.path:
    sys.path.insert(0, my_scripts_dir)
# --- End Path Setup ---

from assignment_2.dmp_controller import *
# Set User-Agent (recommended to avoid potential blocking and for identification)
os.environ["USER_AGENT"] = "MyRAGSystem/0.1 (e11806417@student.tuwien.ac.at or project-url)"

def load_documents(sources: List[str]) -> List[Document]:
    """
    Load documents from multiple sources, which can be either URLs or local file paths.
    
    Args:
        sources: List of URLs or local file paths
        
    Returns:
        List of loaded documents
    """
    all_documents = []
    
    for source in sources:
        try:
            if source.startswith(('http://', 'https://')):
                # Handle web URLs
                print(f"Loading web source: {source}")
                if source.endswith('.pdf'):
                    # Handle PDF URLs
                    loader = PyPDFLoader(source)
                else:
                    # Handle regular web pages
                    loader = WebBaseLoader(source)
                documents = loader.load()
                print(f"Loaded {len(documents)} document(s) from {source}")
            else:
                # Handle local files
                if os.path.isfile(source):
                    if source.endswith('.html'):
                        print(f"Loading local HTML file: {source}")
                        loader = BSHTMLLoader(source)
                        documents = loader.load()
                    elif source.endswith('.pdf'):
                        print(f"Loading local PDF file: {source}")
                        loader = PyPDFLoader(source)
                        documents = loader.load()
                    else:
                        print(f"Unsupported file type: {source}")
                        continue
                    print(f"Loaded {len(documents)} document(s) from {source}")
                elif os.path.isdir(source):
                    # Handle directory of HTML and PDF files
                    print(f"Loading HTML/PDF files from directory: {source}")
                    html_files = glob.glob(os.path.join(source, "*.html"))
                    pdf_files = glob.glob(os.path.join(source, "*.pdf"))
                    documents = []
                    
                    # Process HTML files
                    for html_file in html_files:
                        print(f"  Loading HTML: {html_file}")
                        loader = BSHTMLLoader(html_file)
                        documents.extend(loader.load())
                    
                    # Process PDF files
                    for pdf_file in pdf_files:
                        print(f"  Loading PDF: {pdf_file}")
                        loader = PyPDFLoader(pdf_file)
                        documents.extend(loader.load())
                        
                    print(f"Loaded {len(documents)} document(s) from directory {source}")
                else:
                    print(f"Unsupported file type or not found: {source}")
                    continue
            
            all_documents.extend(documents)
        except Exception as e:
            print(f"Error loading from {source}: {e}")
    
    return all_documents

def decode_move_tag(tag: str) -> dict:
    """
    Decodes a Tower of Hanoi move tag into a dictionary.

    Args:
        tag: The move tag string (e.g., "MD1AC").

    Returns:
        A dictionary with 'disk', 'start', and 'end' keys.
        Returns None if the tag format is invalid.
    """
    match = re.fullmatch(r"MD(\d+)([A-C])([A-C])", tag)
    if match:
        disk_number = int(match.group(1))
        source_peg = match.group(2)
        destination_peg = match.group(3)
        return {"disk": disk_number, "start": source_peg, "end": destination_peg}
    else:
        print(f"Warning: Invalid tag format encountered: {tag}")
        return None


def motion_sequence(grasper_node,cube_to_grasp, lift_target_pqs, target_pqs, subsample_factor_ik):
    motion_sequence = [
                {'name': 'pick', 'action': lambda: grasper_node.execute_motion(motion_type='place' ,subsample_factor_ik=subsample_factor_ik, wait_for_tf_sec=2.0)},
                {'name': 'lift', 'action': lambda: grasper_node.lift_cube(goal_pose_pqs=lift_target_pqs, subsample_factor_ik=subsample_factor_ik)},
                {'name': 'place', 'action': lambda: grasper_node.place_cube(start_pose_pqs=lift_target_pqs, subsample_factor_ik=subsample_factor_ik)},
                {'name': 'home', 'action': lambda: grasper_node.go_home(goal_pose_pqs=lift_target_pqs, subsample_factor_ik=subsample_factor_ik)} 
            ]
    return motion_sequence

def main():

    # -----------------Initialize the DMP Cube Manipulator-----------------#
    
    grasper_node = None
    DMP_FILES = {
        'pick': '/root/catkin_ws/recordings/learned_pick_motion_11.pkl',
        'lift': '/root/catkin_ws/recordings/learned_lift_motion_4.pkl',
        'place': '/root/catkin_ws/recordings/learned_place_motion_14.pkl',
        'home': '/root/catkin_ws/recordings/learned_release_motion_2.pkl' # Renamed from 'home' in example
    }
    URDF_FILE = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/urdf/open_manipulator_6dof.urdf'
    MESH_DIR = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/meshes'
    WORLD_FRAME = "world"

    LIFT_TARGET_PQS = np.array([0.08039667, -0.00823571, 0.12112987, 0.40824577, 0.03776871, 0.91182508, -0.02199852]) 

    

    TF_WAIT_TIME = 1.5
    IK_SUBSAMPLE = 5
    PUB_RATE = 20.0
    TF_RATE = 10.0
    PAUSE_BETWEEN_MOTIONS = 1.0 # Seconds
    JOINT_STATES_TOPIC = "/joint_states" # Default, but can be configured

    grasper_node = DMPCubeManipulator(
        dmp_paths=DMP_FILES,
        urdf_path=URDF_FILE,
        mesh_path=MESH_DIR,
        base_frame=WORLD_FRAME,
        tf_update_rate=TF_RATE,
        publish_rate=PUB_RATE,
        joint_states_topic=JOINT_STATES_TOPIC
    )
    print(f"Grasper node :{grasper_node}")


    print("Initializing Retrieval Augmented Generation system...")

    # Define your sources directly in an array
    # Include a mix of local HTML files and web URLs as needed
    sources = [
        "/workspace/Docker_volume/hanoi_tagged_solution_3_disks.html",
        # "https://www.geeksforgeeks.org/iterative-tower-of-hanoi/"  # Keeping the original source as well
    ]
    
    print("\nUsing the following document sources:")
    for idx, source in enumerate(sources, 1):
        print(f"{idx}. {source}")
    
    # 1. Load Data
    print("\nLoading documents from sources...")
    data = load_documents(sources)
    if not data:
        print("Error: No data loaded from any sources. Please check your URLs and file paths.")
        return
    print(f"Successfully loaded {len(data)} document(s) in total.")

    # 2. Split Data
    print("Splitting documents into manageable chunks...")
    text_splitter = RecursiveCharacterTextSplitter(chunk_size=500, chunk_overlap=50)
    all_splits = text_splitter.split_documents(data)
    print(f"Split documents into {len(all_splits)} chunks.")

    # 3. Initialize Embeddings
    OLLAMA_EMBEDDING_MODEL = "nomic-embed-text"
    print(f"Initializing Ollama embeddings ({OLLAMA_EMBEDDING_MODEL})...")
    try:
        # Ensure you have pulled the nomic-embed-text model in Ollama:
        # ollama pull nomic-embed-text
        embeddings = OllamaEmbeddings(model=OLLAMA_EMBEDDING_MODEL)
        # Test the embedding model
        print("Testing embedding model...")
        _ = embeddings.embed_query("Test query for embedding model.")
        print(f"Ollama embedding model '{OLLAMA_EMBEDDING_MODEL}' initialized successfully.")
    except Exception as e:
        print(f"Error initializing Ollama embeddings with model '{OLLAMA_EMBEDDING_MODEL}': {e}.")
        print(f"Ensure Ollama is running and you have pulled the '{OLLAMA_EMBEDDING_MODEL}' model (e.g., 'ollama pull {OLLAMA_EMBEDDING_MODEL}').")
        print("Also ensure you have installed 'pip install -U langchain-ollama'.")
        return

    # 4. Create Vector Store
    COLLECTION_NAME = "rag-chroma-nomic-embed"
    print(f"Creating ChromaDB vector store (in-memory, collection: {COLLECTION_NAME})...")
    try:
        vectorstore = Chroma.from_documents(
            documents=all_splits,
            collection_name=COLLECTION_NAME,
            embedding=embeddings,
        )
        retriever = vectorstore.as_retriever()
        print("Vector store created successfully.")
    except Exception as e:
        print(f"Error creating vector store: {e}")
        return

    # 5. Setup Ollama LLM for Chat
    OLLAMA_CHAT_MODEL = 'gemma3:4b' # Or your preferred Ollama chat model like 'llama3', 'mistral'
    print(f"Using Ollama chat model: {OLLAMA_CHAT_MODEL}")
    try:
        model_local = ChatOllama(model=OLLAMA_CHAT_MODEL)
        # Perform a quick test invocation to check connectivity
        print(f"Testing Ollama chat model '{OLLAMA_CHAT_MODEL}' connection...")
        _ = model_local.invoke("This is a test prompt.")
        print(f"Ollama chat model '{OLLAMA_CHAT_MODEL}' connection successful.")
    except Exception as e:
        print(f"Error connecting to Ollama chat model '{OLLAMA_CHAT_MODEL}': {e}")
        print(f"Please ensure Ollama is running and the specified model is available (e.g., 'ollama pull {OLLAMA_CHAT_MODEL}').")
        print("Also ensure you have installed 'pip install -U langchain-ollama'.")
        return

   
    template = """Answer the question based on the following context if relevant. If the context does not contain information related to the question, you may use your general knowledge to provide an answer.

    Context:
    {context}

    Question: {question}

    Answer:
    """
    prompt = ChatPromptTemplate.from_template(template)

    # 7. Define RAG Chain
    chain = (
        {"context": retriever, "question": RunnablePassthrough()}
        | prompt
        | model_local
        | StrOutputParser()
    )
    print("RAG chain is configured and ready.")
    print("\n--- Starting Question Answering Session ---")
    print("Type 'quit', 'exit', or 'q' to end the session.")


    prompt = f"Solve the Tower of Hanoi problem with 3 disks. Return the solution only in tagged format"
    print("Processing the prompt...")
    answer = chain.invoke(prompt)
    print(f"Answer received: {answer}")

    line = answer.split('\n')[0]  # Assuming the first line contains the moves
    if line.startswith("MD"):
        move = decode_move_tag(line)
        if move:
            print(f"Decoded Move: Disk {move['disk']} from {move['start']} to {move['end']}")
            sequence = motion_sequence(
                grasper_node=grasper_node,
                cube_to_grasp=f"cube_{move['disk']}",
                lift_target_pqs=LIFT_TARGET_PQS,
                target_pqs=np.array([0.08039667, -0.00823571, 0.12112987, 0.40824577, 0.03776871, 0.91182508, -0.02199852]),
                subsample_factor_ik=IK_SUBSAMPLE
            )
            for motion_info in sequence:
                rospy.loginfo(f"Executing: {motion_info['name']}")
                results = motion_info['action']()
                if results is None or results[0] is None: # results[0] is joint_traj_arm
                    raise RuntimeError(f"{motion_info['name'].capitalize()} motion failed.")
                joint_traj_arm, gripper_traj, time_stamps, cartesian_traj_viz = results
                print(f"Grasper node: {grasper_node}")
                # grasper_node.simulate_trajectory(motion_info['name'], joint_traj_arm, cartesian_traj_viz) # Optional
                # grasper_node.publish_trajectory(joint_traj_arm, time_stamps, gripper_traj) # UNCOMMENTED for actual execution
                rospy.loginfo(f"Waiting {PAUSE_BETWEEN_MOTIONS}s after {motion_info['name']}...")
                rospy.sleep(PAUSE_BETWEEN_MOTIONS)
        else:
            print(f"Invalid move tag encountered: {line}")
    else:
        print(f"Non-move line: {line}")

if __name__ == "__main__":
    main()
# combined_rag_toh.py
# Import functions directly from the existing files
import re
from rag_call import load_documents, OllamaEmbeddings, RecursiveCharacterTextSplitter
from rag_call import Chroma, ChatPromptTemplate, ChatOllama, RunnablePassthrough, StrOutputParser
from toh_visual import TowerOfHanoi, parse_and_move
from matplotlib import pyplot as plt

def setup_rag_system(sources):
    """Set up the RAG system using functions from rag_call.py"""
    print("\nSetting up the RAG system...")
    
    # 1. Load Data
    print("Loading documents from sources...")
    data = load_documents(sources)
    if not data:
        print("Error: No data loaded from any sources. Please check your URLs and file paths.")
        return None
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
        embeddings = OllamaEmbeddings(model=OLLAMA_EMBEDDING_MODEL)
        # Test the embedding model
        _ = embeddings.embed_query("Test query for embedding model.")
        print(f"Ollama embedding model '{OLLAMA_EMBEDDING_MODEL}' initialized successfully.")
    except Exception as e:
        print(f"Error initializing Ollama embeddings: {e}")
        return None

    # 4. Create Vector Store
    COLLECTION_NAME = "rag-chroma-nomic-embed"
    print(f"Creating ChromaDB vector store...")
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
        return None

    # 5. Setup Ollama LLM for Chat
    OLLAMA_CHAT_MODEL = 'gemma3:4b'  # Or your preferred Ollama chat model
    print(f"Using Ollama chat model: {OLLAMA_CHAT_MODEL}")
    try:
        model_local = ChatOllama(model=OLLAMA_CHAT_MODEL)
        # Perform a quick test invocation
        _ = model_local.invoke("This is a test prompt.")
        print(f"Ollama chat model '{OLLAMA_CHAT_MODEL}' connection successful.")
    except Exception as e:
        print(f"Error connecting to Ollama chat model: {e}")
        return None

    # 6. Define Prompt Template
    # template = """Answer the question based *only* on the following context. If the context does not contain the answer, state that you do not have enough information from the provided context to answer. Do not make up information.

    # Context:
    # {context}

    # Question: {question}

    # Answer:
    # """
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
    
    return chain

def visualize_toh_solution(moves):
    """Visualize Tower of Hanoi solution using functions from toh_visual.py"""
    print("\nVisualizing Tower of Hanoi solution...")
    
    # Extract the number of disks from the move codes (assuming format MDxYZ where x is disk number)
    disk_numbers = [int(move[2]) for move in moves if len(move) >= 3 and move[2].isdigit()]
    num_disks = max(disk_numbers) if disk_numbers else 3
    
    # Initialize Tower of Hanoi visualization
    hanoi = TowerOfHanoi(num_disks)
    plt.pause(1)  # Wait for user to see initial state
    
    # Execute each move
    all_moves_successful = True
    for move in moves:
        if not parse_and_move(hanoi, move):
            print(f"Stopping due to invalid move: {move}")
            all_moves_successful = False
            break
    
    print("\nVisualization complete.")
    if all_moves_successful and len(hanoi.pegs['C']) == hanoi.num_disks and not hanoi.pegs['A'] and not hanoi.pegs['B']:
        print("Puzzle solved correctly!")
    else:
        print("Puzzle not fully solved.")
        print(f"Final peg states: A: {hanoi.pegs['A']}, B: {hanoi.pegs['B']}, C: {hanoi.pegs['C']}")
    
    plt.ioff()
    plt.show()

def extract_toh_moves(answer_text):
    """Extract Tower of Hanoi moves from the answer text"""
    # Look for move patterns like MD1AC, MD2AB, etc.
    move_pattern = r'MD\d[A-C][A-C]'
    moves = re.findall(move_pattern, answer_text)
    
    # If no direct pattern matches, look for numbered list of moves
    if not moves:
        # Try to find moves in a numbered list format (e.g., "1. MD1AC")
        numbered_pattern = r'\d+\.\s*(MD\d[A-C][A-C])'
        moves = re.findall(numbered_pattern, answer_text)
    
    return moves

def main():
    # Define your document sources
    sources = [
        "/workspace/Docker_volume/hanoi_tagged_solution_3_disks.html",
        "https://en.wikipedia.org/wiki/Tower_of_Hanoi"
        # Add other sources as needed
    ]
    
    # Set up the RAG system
    chain = setup_rag_system(sources)
    if chain is None:
        print("Failed to set up RAG system. Exiting.")
        return
    
    print("\n--- Starting Question Answering Session ---")
    print("Type 'quit', 'exit', or 'q' to end the session.")
    
    while True:
        user_question = input("\nEnter your question: ")
        if user_question.lower() in ["quit", "exit", "q"]:
            print("Exiting system.")
            break
        if not user_question.strip():
            print("No question entered. Please provide a question.")
            continue
        
        print("Processing your question...")
        try:
            answer = chain.invoke(user_question)
            print("\nAnswer:")
            print(answer)
            
            # Check if the answer contains Tower of Hanoi moves
            moves = extract_toh_moves(answer)
            if moves:
                print(f"\nDetected Tower of Hanoi moves: {', '.join(moves)}")
                visualize = input("Would you like to visualize these moves? (yes/no): ")
                if visualize.lower() in ["yes", "y"]:
                    visualize_toh_solution(moves)
            
        except Exception as e:
            print(f"Error during processing: {e}")

if __name__ == "__main__":
    # Ensure Ollama is installed and running with required models:
    # - nomic-embed-text
    # - gemma3:4b (or your chosen model)
    main()